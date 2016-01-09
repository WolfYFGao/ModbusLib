using System;
using System.IO;
using System.IO.Ports;
using System.Threading;
#if NETMF
using Microsoft.SPOT.Hardware;
using Microsoft.SPOT;
using System.Text;
#endif

namespace Osre.Modbus.Interface
{
   /// <summary>
   /// ModbusRtuInterface is a Modbus RTU implemention of the <see cref="IModbusInterface"/> interface 
   /// to be used with <see cref="ModbusMaster"/> or <see cref="ModbusDevice"/>.
   /// </summary>
   public class ModbusAsciiInterface : IModbusInterface
   {
      private readonly SerialPort _serial;
#if NETMF
      private readonly OutputPort _writePort;
      private readonly bool _writeState;
      private readonly bool _isMaster;
#endif
      private readonly long _halfCharLength;
      private long _nextSend;

      /// <summary>
      /// Gets the serial port which is used.
      /// </summary>
      public SerialPort SerialPort
      {
         get { return _serial; }
      }

#if NETMF
      /// <summary>
      /// Creates a new Modbus RTU interface using an existing and fully initialized <see cref="SerialPort"/>.
      /// This interface can be used with <see cref="ModbusMaster"/> or <see cref="ModbusDevice"/>.
      /// </summary>
      /// <param name="serial">Fully initialized <see cref="SerialPort"/>.</param>
      /// <param name="writePin">Pin for select read/write mode. Set <see cref="Cpu.Pin.GPIO_NONE"/> for an automatic switching transceiver.</param>
      /// <param name="writeState">State of writePin to set for writing.</param>
      /// <param name="maxDataLength">Maximum number of data bytes</param>
      /// <remarks>
      /// This overload allows to specify a digital output pin by which read/write is selected.
      /// For writing the value of writeState is set, For reading !writeState is set.
      /// The pin is held at state read for all the time except in the short write phase.
      /// </remarks>
      public ModbusAsciiInterface(SerialPort serial, Cpu.Pin writePin, bool writeState, short maxDataLength = 252) :
         this(serial, maxDataLength)
      {
         _writeState = writeState;
         if (writePin != Cpu.Pin.GPIO_NONE)
         {
            // initialize output port for selecting read/write and initialize it for reading
            _writePort = new OutputPort(writePin, !writeState);
         }
      }
#endif

      /// <summary>
      /// Creates a new Modbus RTU interface using an existing and fully initialized <see cref="SerialPort"/>.
      /// This interface can be used with <see cref="ModbusMaster"/> or <see cref="ModbusDevice"/>.
      /// </summary>
      /// <param name="serial">Fully initialized <see cref="SerialPort"/>.</param>
      /// <param name="maxDataLength">Maximum number of data bytes</param>
      public ModbusAsciiInterface(SerialPort serial, short maxDataLength = 252)
      {
         if (serial.DataBits < 8)
         {
            throw new ArgumentException("serial.DataBits must be >= 8");
         }
         _serial = serial;
         MaxDataLength = maxDataLength;
         MaxTelegramLength = (short)(maxDataLength + 4);

         // calc char length in µs
         if (serial.BaudRate > 19200)
         {
            // use a fixed value for high baudrates (recommended by Modbus spec.)
            _halfCharLength = 500;
         }
         else
         {
            short bitCnt = (short)serial.DataBits;
            switch (serial.StopBits)
            {
               case StopBits.One:
                  ++bitCnt;
                  break;

               case StopBits.OnePointFive:
               case StopBits.Two:
                  bitCnt += 2;
                  break;
            }
            if (serial.Parity != Parity.None)
            {
               ++bitCnt;
            }
            _halfCharLength = (short)((bitCnt * 1000 * 10000) / serial.BaudRate) >> 1;
         }
      }

      /// <summary>
      /// Gets the maximum data length (not including address, function code, ...) of e telegram.
      /// </summary>
      public short MaxDataLength { get; private set; }

      /// <summary>
      /// Gets the maximum length of a Modbus telegram.
      /// </summary>
      public short MaxTelegramLength { get; private set; }

      /// <summary>
      /// Creates a new telegram for a modbus request or response.
      /// All data except the function code specific user data is written into the given buffer.
      /// </summary>
      /// <param name="addr">Device address. 0 = Breadcast, 1..247 are valid device addresses.</param>
      /// <param name="fkt">Function code. <see cref="ModbusFunctionCode"/></param>
      /// <param name="dataLength">Number of bytes for function code sspecific user data.</param>
      /// <param name="buffer">Buffer to write data into. The buffer must be at least MaxTelegramLength - MaxDataLength + dataLength bytes long.</param>
      /// <param name="telegramLength">Returns the total length of the telegram in bytes.</param>
      /// <param name="dataPos">Returns the offset of the function code specific user data in buffer.</param>
      /// <param name="isResponse">true if this is a response telegram; false if this is a request telegram.</param>
      /// <param name="telegramContext">
      /// If isResponse == false, this parameter returns the interface implementation specific data which must be passed to the ParseTelegram method of the received response.
      /// If isResponse == true, this parameter must be called with the telegramContext parameter returned by ParseTelegram of the request telegram.</param>
      public void CreateTelegram(byte addr, byte fkt, short dataLength, byte[] buffer, out short telegramLength, out short dataPos, bool isResponse, ref object telegramContext)
      {
          telegramLength = (short)(4 + dataLength);
          dataPos = 2;
          buffer[0] = addr;
          buffer[1] = fkt;
      }

      public void PrepareWrite()
      {
#if NETMF
         if (_writePort != null)
         {
            // switch tp write mode
            _writePort.Write(_writeState);
         }
#endif
      }

      public void PrepareRead()
      {
#if NETMF
         if (_writePort != null)
         {
            // switch tp write mode
            _writePort.Write(!_writeState);
         }
#endif
      }

      /// <summary>
      /// Sends the given telegram.
      /// If necessary additional information like a checksum can be inserted here.
      /// </summary>
      /// <param name="buffer">Buffer containing the data.</param>
      /// <param name="telegramLength">Length of the telegram in bytes.</param>
      public void SendTelegram(byte[] buffer, short telegramLength)
      {         
         // Create ASCII telegram
         string ascii_telegram = "";
         for (int i = 0; i < telegramLength - 2; i++)
         {
             ascii_telegram += ModbusUtils.Hex((int) buffer[i]);
         }
         System.Text.UTF8Encoding encoder = new System.Text.UTF8Encoding();
         byte[] bytesToSend = encoder.GetBytes(ascii_telegram);
#if NETMF
         Debug.Print("TelegramLength: " + telegramLength.ToString());
#endif
          var lrc = ModbusUtils.CalcLrc(buffer, 0, (telegramLength - 2));

         ascii_telegram = ":" + ascii_telegram + ModbusUtils.Hex(lrc) + "\r\n";
         encoder = new System.Text.UTF8Encoding();
         bytesToSend = encoder.GetBytes(ascii_telegram);
#if NETMF
          Debug.Print(ascii_telegram);
#endif

         // ticks and _NextSend are multiples of 100 ns
         var dt = _nextSend - DateTime.Now.Ticks;
         if (dt > 0)
         {
            Thread.Sleep(System.Math.Max(1, (int)dt / 10000));
         }

         // clear buffers
         _serial.DiscardInBuffer();
         _serial.DiscardOutBuffer();

         // next send is 3.5 chars after the end of this telegram
         _nextSend = DateTime.Now.Ticks + (telegramLength * 2 + 7) * _halfCharLength;
#if NETMF
         try
         {
             _serial.Write(bytesToSend, 0, bytesToSend.Length);
            // make sure all bytes are sent out
            _serial.Flush();
         }
         catch (ArgumentException)
         {
            // happens some times -> Close -> Open -> try again
            try
            {
               _serial.Close();
            }
            catch
            {
               // ignored
            }
            _serial.Open();
            _serial.Write(bytesToSend, 0, bytesToSend.Length);
            _serial.Flush();
         }

#else
         _serial.Write(buffer, 0, telegramLength);
#endif
      }

      /// <summary>
      /// Waits and receives a telegram.
      /// </summary>
      /// <param name="buffer">Buffer to write data into.</param>
      /// <param name="desiredDataLength">Desired length of the function code specific data in bytes. -1 if length is unknown.</param>
      /// <param name="timeout">Timeout in milliseconds to wait for the telegram.</param>
      /// <param name="telegramLength">Returns the total length of the telegram in bytes.</param>
      /// <returns>Returns true if the telegram was received successfully; false on timeout.</returns>
      public bool ReceiveTelegram(byte[] buffer, short desiredDataLength, int timeout, out short telegramLength)
      {
         short desiredLength;
         if (desiredDataLength >= 0)
         {
            desiredLength = (short)((desiredDataLength * 2) + 9);
#if NETMF
                Debug.Print("desiredlength = " + desiredLength.ToString());
#endif
             if (desiredLength > buffer.Length)
            {
#if NETMF
                Debug.Print("desiredlength > buffer.length");
#endif
               throw new ArgumentException(String.Concat("buffer size (" , buffer.Length ,") must be at least 9 byte larger than desiredDataLength*2 (", desiredDataLength, ")"));
            }
         }
         else
         {
            desiredLength = -1;
         }

         int n = 0;
         var tOut = DateTime.Now.AddMilliseconds(timeout);
         long nextRead = 0;
         bool errorChecked = false;
         while (true)
         {
            //if ((desiredLength > 0 || n == 0) && DateTime.Now > tOut)
            if (DateTime.Now > tOut)
            {
               break;
            }
            if (_serial.BytesToRead > 0)
            {
               if (desiredLength > 0)
               {
                  n += _serial.Read(buffer, n, desiredLength - n);
               }
               else
               {
                  n += _serial.Read(buffer, n, buffer.Length - n);
               }
#if NETMF
                Debug.Print(new string(Encoding.UTF8.GetChars(buffer, 0, n)));
#endif
               // ASCII has 1 second nextRead limit
               nextRead = DateTime.Now.Ticks + 33333 * _halfCharLength;
            }
            if (!errorChecked && n >= 5)
            {
               errorChecked = true;
               if ((buffer[3] == 0x38) && (buffer[4] == 0x30))
               {
                  // modbus error, so desired length is 11
                  desiredLength = 11;
               }
            }
            if (desiredLength > 0 && n == desiredLength)
            {
#if NETMF
                Debug.Print("Read n chars: " + n.ToString() + " on " + desiredLength.ToString());
#endif
                telegramLength = (short)n;
                return true;
            }
            if (desiredLength <= 0 && n >= 4 && DateTime.Now.Ticks > nextRead && _serial.BytesToRead == 0)
            {
                // Check packet
                if (buffer[0] == 0x3A && buffer[n - 2] == 0x0D && buffer[n - 1] == 0x0A)
                {
#if NETMF
                    Debug.Print("Read n chars: " + n.ToString() + " on " + desiredLength.ToString());
#endif              
                    telegramLength = (short)n;
                    return true;
                }
                else
                {
                    // read a little bit longer
                    Thread.Sleep(1);
                    nextRead = DateTime.Now.Ticks + 33333 * _halfCharLength;
                }
            }
         }
#if NETMF
          Debug.Print("Timeout reading in Interface.");
#endif         
          telegramLength = 0;
         return false;
      }

      /// <summary>
      /// Parses a telegram received by ReceiveTelegram.
      /// </summary>
      /// <param name="buffer">Buffer containing the data.</param>
      /// <param name="telegramLength">Total length of the telegram in bytes.</param>
      /// <param name="isResponse">true if the telegram is a response telegram; false if the telegram is a request telegram.</param>
      /// <param name="telegramContext">
      /// If isResponse == true: pass the telegramContext returned by CreateTelegram from the request.
      /// If isResponse == false: returns the telegramContext from the received request. It must pe passed to the CreateTelegram method for the response.
      /// </param>
      /// <param name="address">Returns the device address.</param>
      /// <param name="fkt">Returns the function code.</param>
      /// <param name="dataPos">Returns the offset in buffer of the function code specific data.</param>
      /// <param name="dataLength">Returns the length of the function code specific data.</param>
      /// <returns>Returns true if this is the matching response according to the telegramContext; else false. If isResponse == false this method should return always true.</returns>
      public bool ParseTelegram(byte[] buffer, short telegramLength, bool isResponse, ref object telegramContext, out byte address, out byte fkt,
                                out short dataPos, out short dataLength)
      {
          int real_length = telegramLength;
          int virtual_length = 0;
#if NETMF
          Debug.Print("Real length: " + real_length.ToString());
#endif

          if (telegramLength < 11)
          {
#if NETMF
              Debug.Print("Received a Modbus Ascii packet too short.");
#endif
              throw new ModbusException(ModbusErrorCode.ResponseTooShort);
          }

          if (buffer[0] != 0x3A || buffer[real_length-2] != 0x0D || buffer[real_length-1] != 0x0A)
          {
#if NETMF
              Debug.Print("Received not a good Modbus Ascii packet.");
#endif
              throw new ModbusException(ModbusErrorCode.Unspecified);
          }

          for (int i = 1; i < real_length - 2; i += 2)
          {
#if NETMF
              Debug.Print("Char: " + ModbusUtils.ExtractAsciiAsByte(buffer, i).ToString());
#endif
              buffer[(i - 1) / 2] = ModbusUtils.ExtractAsciiAsByte(buffer, i);
          }
          virtual_length = (real_length - 1) / 2;
#if NETMF
          Debug.Print("Virtual length: " + virtual_length.ToString());
#endif
          var lrc = ModbusUtils.CalcLrc(buffer, 0, virtual_length - 2);
          if (buffer[virtual_length - 2] != lrc)
         {
#if NETMF
             Debug.Print("Modbus Ascii LRC ERROR.");
#endif
             throw new ModbusException(ModbusErrorCode.CrcError);
         }

         address = buffer[0];
         fkt = buffer[1];
         dataPos = 2;
         dataLength = (short)(virtual_length - 4);
         return true;
      }

      /// <summary>
      /// Gets if there is currently dataavailable on the interface.
      /// </summary>
      public bool IsDataAvailable
      {
         get { return _serial.BytesToRead > 0; }
      }

      /// <summary>
      /// Removes all data from the input interface.
      /// </summary>
      public void ClearInputBuffer()
      {
         _serial.DiscardInBuffer();
      }

      /// <summary>
      /// Gets if the connection is ok
      /// </summary>
      public bool IsConnectionOk
      {
         get { return _serial != null && _serial.IsOpen; }
      }
   }
}
