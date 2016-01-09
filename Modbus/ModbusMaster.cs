using System;
using System.Collections;
using Osre.Modbus.Interface;

// ReSharper disable TooWideLocalVariableScope
namespace Osre.Modbus
{
   /// <summary>
   /// ModbusMaster provides the function code implementaions for a modbus master.
   /// </summary>
   /// <remarks>
   /// A Modbus Interface is required for communication.
   /// Use <see cref="Interface.ModbusRtuInterface"/> or <see cref="Interface.ModbusTcpInterface"/>.
   /// </remarks>
   public class ModbusMaster
   {
      private readonly IModbusInterface _Interface;
      private readonly object _SyncObject;
      private readonly byte[] _Buffer;

      /// <summary>
      /// Creates a modbus master using the given interface
      /// </summary>
      /// <param name="intf">Interface to use for the master</param>
      /// <param name="syncObject">Object to use for multi threaded communication locking.</param>
      public ModbusMaster(IModbusInterface intf, object syncObject = null)
      {
         _Interface = intf;
         _SyncObject = syncObject ?? new object();
         _Buffer = new byte[_Interface.MaxTelegramLength];
         _Interface.PrepareWrite();
      }

      /// <summary>
      /// Sends a modbus telegram over the interface and waits for a response
      /// </summary>
      /// <param name="deviceAddress">Modbus device address.</param>
      /// <param name="fc">Function code.</param>
      /// <param name="timeout">Timeout in Milli seconds.</param>
      /// <param name="telegramLength">Total length of the telegram in bytes</param>
      /// <param name="desiredDataLength">Length of the desired telegram data (without fc, cs, ...) of the response in bytes. -1 for unknown.</param>
      /// <param name="telegramContext">Interface specific context of the telegram</param>
      /// <param name="dataPos">Index of the response data in the buffer.</param>
      /// <returns></returns>
      protected virtual short SendReceive(byte deviceAddress, ModbusFunctionCode fc, int timeout, short telegramLength, short desiredDataLength,
                                          object telegramContext, ref short dataPos)
      {
         lock (_SyncObject)
         {
            try
            {
               _Interface.SendTelegram(_Buffer, telegramLength);
               _Interface.PrepareRead();

               if (deviceAddress == ModbusConst.BroadcastAddress)
               {
                  return 0;
               }

               byte responseDeviceAddress;
               byte responseFc = 0;
               short dataLength = 0;

               while (timeout > 0)
               {
                  var ts = DateTime.Now.Ticks;
                  if (!_Interface.ReceiveTelegram(_Buffer, desiredDataLength, timeout, out telegramLength))
                  {
                     throw new ModbusException(ModbusErrorCode.Timeout);
                  }
                  timeout -= (int) ((DateTime.Now.Ticks - ts)/10000);

                  // if this is not the response we are waiting for wait again until time runs out
                  if (_Interface.ParseTelegram(_Buffer, telegramLength, true, ref telegramContext,
                     out responseDeviceAddress,
                     out responseFc,
                     out dataPos, out dataLength)
                      && responseDeviceAddress == deviceAddress
                      && (responseFc & 0x7f) == (byte) fc)
                  {
                     break;
                  }
                  if (timeout <= 0)
                  {
                     throw new ModbusException(ModbusErrorCode.Timeout);
                  }
               }
               if ((responseFc & 0x80) != 0)
               {
                  // error response
                  throw new ModbusException((ModbusErrorCode) _Buffer[dataPos]);
               }
               return dataLength;
            }
            finally
            {
               _Interface.PrepareWrite();
            }
         }
      }

      /// <summary>
      /// Sends a Read Coils (0x01) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="startAddress">Start address: 0x0000 .. 0xFFFF</param>
      /// <param name="coilCount">>Number of coils to read: 1 .. 2000</param>
      /// <param name="timeout">Timeout for response in Milli seconds.</param>
      /// <returns>Returns a byte array which contains the coils. The coils are written as single bits into the array starting with coil 1 at the lsb.</returns>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual byte[] ReadCoils(byte deviceAddress, ushort startAddress, ushort coilCount, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;
         
         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.ReadCoils, 4, 
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         ModbusUtils.InsertUShort(_Buffer, dataPos, startAddress);
         ModbusUtils.InsertUShort(_Buffer, dataPos + 2, coilCount);

         short desiredDataLength = (short)(1 + coilCount / 8);
         if (coilCount%8 != 0)
         {
            ++desiredDataLength;
         }

         var dataLength = SendReceive(deviceAddress, ModbusFunctionCode.ReadCoils, timeout, telegramLength, desiredDataLength, telegramContext, ref dataPos);

         if (deviceAddress == ModbusConst.BroadcastAddress)
         {
            return null;
         }

         var coils = new byte[dataLength - 1];
         Array.Copy(_Buffer, dataPos + 1, coils, 0, dataLength - 1);
         return coils;
      }

      /// <summary>
      /// Sends a Read Discrete Inputs (0x02) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="startAddress">Start address: 0x0000 .. 0xFFFF</param>
      /// <param name="inputCount">>Number of coils to read: 1 .. 2000</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <returns>Returns a byte array which contains the inputs. The inputs are written as single bits into the array starting with coil 1 at the lsb.</returns>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual byte[] ReadDiscreteInputs(byte deviceAddress, ushort startAddress, ushort inputCount, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.ReadDiscreteInputs, 4,
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         ModbusUtils.InsertUShort(_Buffer, dataPos, startAddress);
         ModbusUtils.InsertUShort(_Buffer, dataPos + 2, inputCount);

         short desiredDataLength = (short)(1 + inputCount / 8);
         if (inputCount % 8 != 0)
         {
            ++desiredDataLength;
         }

         var dataLength = SendReceive(deviceAddress, ModbusFunctionCode.ReadDiscreteInputs, timeout, telegramLength, desiredDataLength, telegramContext, ref dataPos);

         if (deviceAddress == ModbusConst.BroadcastAddress)
         {
            return null;
         }

         var coils = new byte[dataLength - 1];
         Array.Copy(_Buffer, dataPos + 1, coils, 0, dataLength - 1);
         return coils;
      }

      /// <summary>
      /// Sends a Read Holding Registers (0x03) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="startAddress">Start address: 0x0000 .. 0xFFFF</param>
      /// <param name="registerCount">Number of the registers to read.</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <returns>Returns an array of 16 bit register values.</returns>
      /// <remarks>
      /// A register is a 16 bit unsigned value.
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual ushort[] ReadHoldingRegisters(byte deviceAddress, ushort startAddress, ushort registerCount, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.ReadHoldingRegisters, 4,
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         ModbusUtils.InsertUShort(_Buffer, dataPos, startAddress);
         ModbusUtils.InsertUShort(_Buffer, dataPos + 2, registerCount);

         short desiredDataLength = (short)(1 + 2 * registerCount);

         var dataLength = SendReceive(deviceAddress, ModbusFunctionCode.ReadHoldingRegisters, timeout, telegramLength, desiredDataLength, telegramContext, ref dataPos);

         if (deviceAddress == ModbusConst.BroadcastAddress)
         {
            return null;
         }

         var registers = new ushort[(dataLength - 1) / 2];
         for (int n = 0; n < registers.Length; ++n)
         {
            registers[n] = ModbusUtils.ExtractUShort(_Buffer, dataPos + 1 + 2 * n);
         }
         return registers;
      }

      /// <summary>
      /// Sends a Read Input Registers (0x04) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="startAddress">Start address: 0x0000 .. 0xFFFF</param>
      /// <param name="registerCount">Number of the registers to read.</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <returns>Returns an array of 16 bit register values.</returns>
      /// <remarks>
      /// A register is a 16 bit unsigned value.
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual ushort[] ReadInputRegisters(byte deviceAddress, ushort startAddress, ushort registerCount, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.ReadInputRegisters, 4,
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         ModbusUtils.InsertUShort(_Buffer, dataPos, startAddress);
         ModbusUtils.InsertUShort(_Buffer, dataPos + 2, registerCount);

         short desiredDataLength = (short)(1 + 2 * registerCount);

         var dataLength = SendReceive(deviceAddress, ModbusFunctionCode.ReadInputRegisters, timeout, telegramLength, desiredDataLength, telegramContext, ref dataPos);
         if (deviceAddress == ModbusConst.BroadcastAddress)
         {
            return null;
         }

         var registers = new ushort[(dataLength - 1) / 2];
         for (int n = 0; n < registers.Length; ++n)
         {
            registers[n] = ModbusUtils.ExtractUShort(_Buffer, dataPos + 1 + 2 * n);
         }
         return registers;
      }

      /// <summary>
      /// Sends a Write Single Coils (0x05) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="address">Address of the coil: 0x0000 .. 0xFFFF</param>
      /// <param name="value">true or false</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual void WriteSingleCoil(byte deviceAddress, ushort address, bool value, int timeout = 2000)
      {
         WriteSingleCoil(deviceAddress, address, (ushort)(value ? 0xff00 : 0x0000), timeout);
      }

      /// <summary>
      /// Sends a Write Single Coils (0x05) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="address">Address of the coil: 0x0000 .. 0xFFFF</param>
      /// <param name="value">0xFF00 for true or 0x0000 for false</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual void WriteSingleCoil(byte deviceAddress, ushort address, ushort value, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.WriteSingleCoil, 4,
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         ModbusUtils.InsertUShort(_Buffer, dataPos, address);
         ModbusUtils.InsertUShort(_Buffer, dataPos + 2, value);

         SendReceive(deviceAddress, ModbusFunctionCode.WriteSingleCoil, timeout, telegramLength, 4, telegramContext, ref dataPos);
      }

      /// <summary>
      /// Sends a Write Single Register (0x06) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="address">Address of the register: 0x0000 .. 0xFFFF</param>
      /// <param name="value">Register value to write</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual void WriteSingleRegister(byte deviceAddress, ushort address, ushort value, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.WriteSingleRegister, 4,
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         ModbusUtils.InsertUShort(_Buffer, dataPos, address);
         ModbusUtils.InsertUShort(_Buffer, dataPos + 2, value);

         SendReceive(deviceAddress, ModbusFunctionCode.WriteSingleRegister, timeout, telegramLength, 4, telegramContext, ref dataPos);
      }

      /// <summary>
      /// Sends a Read Exception Status (0x07) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <returns>Returns 8 bit encoded exception statuses of the device.</returns>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual byte ReadExceptionStatus(byte deviceAddress, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.ReadExceptionStatus, 0,
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         SendReceive(deviceAddress, ModbusFunctionCode.ReadExceptionStatus, timeout, telegramLength, 1, telegramContext, ref dataPos);
         if (deviceAddress == ModbusConst.BroadcastAddress)
         {
            return 0;
         }
         return _Buffer[dataPos];
      }

      /// <summary>
      /// Sends a Diagnostics (0x08) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="subFunction">Sub function code.</param>
      /// <param name="data">Data</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <returns>Returns a unsigned short array with diagnostic data.</returns>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual ushort[] Diagnostics(byte deviceAddress, ushort subFunction, ushort[] data, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.Diagnostics, (short)(2 + 2 * data.Length),
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         ModbusUtils.InsertUShort(_Buffer, dataPos, subFunction);
         for (int n = 0; n < data.Length; ++n)
         {
            ModbusUtils.InsertUShort(_Buffer, dataPos + 2 + 2 * n, data[n]);
         }

         var dataLength = SendReceive(deviceAddress, ModbusFunctionCode.Diagnostics, timeout, telegramLength, (short)(2 + 2 * data.Length), telegramContext, ref dataPos);
         if (deviceAddress == ModbusConst.BroadcastAddress)
         {
            return null;
         }
         data = new ushort[(dataLength - 2) / 2];
         for (int n = 0; n < data.Length; ++n)
         {
            data[n] = ModbusUtils.ExtractUShort(_Buffer, dataPos + 2 + 2 * n);
         }

         return data;
      }

      /// <summary>
      /// Sends a Get Comm Event Counter (0x0B) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="status">Receives the status word of the device.</param>
      /// <param name="eventCount">Receives the event counter of the device.</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual void GetCommEventCounter(byte deviceAddress, out ushort status, out ushort eventCount, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.GetCommEventCounter, 0,
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         SendReceive(deviceAddress, ModbusFunctionCode.GetCommEventCounter, timeout, telegramLength, 4, telegramContext, ref dataPos);
         if (deviceAddress == ModbusConst.BroadcastAddress)
         {
            status = 0;
            eventCount = 0;
            return;
         }
         status = ModbusUtils.ExtractUShort(_Buffer, dataPos);
         eventCount = ModbusUtils.ExtractUShort(_Buffer, dataPos + 2);
      }

      /// <summary>
      /// Sends a Get Comm Event Log (0x0C) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="status">Receives the status word of the device.</param>
      /// <param name="eventCount">Receives the event counter of the device.</param>
      /// <param name="messageCount">Receives the event message count.</param>
      /// <param name="events">Receives 0 - 64 bytes, with each byte corresponding to the status of one MODBUS send or receive operation for the remote device.
      /// Byte 0 is the most recent.</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual void GetCommEventLog(byte deviceAddress, out ushort status, out ushort eventCount, out ushort messageCount, out byte[] events, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.GetCommEventLog, 0,
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         SendReceive(deviceAddress, ModbusFunctionCode.GetCommEventLog, timeout, telegramLength, -1, telegramContext, ref dataPos);
         if (deviceAddress == ModbusConst.BroadcastAddress)
         {
            status = 0;
            eventCount = 0;
            messageCount = 0;
            events = null;
            return;
         }
         var byteCnt = _Buffer[dataPos];
         status = ModbusUtils.ExtractUShort(_Buffer, dataPos + 1);
         eventCount = ModbusUtils.ExtractUShort(_Buffer, dataPos + 3);
         messageCount = ModbusUtils.ExtractUShort(_Buffer, dataPos + 5);
         events = new byte[byteCnt - 6];
         Array.Copy(_Buffer, dataPos + 7, events, 0, events.Length);
      }

      /// <summary>
      /// Sends a Write Multiple Coils (0x0F) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="startAddress">Address of the 1st coil: 0x0000 .. 0xFFFF</param>
      /// <param name="coilCount">Number of coils to write.</param>
      /// <param name="coils">Byte array with bit coded coil values.</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual void WriteMultipleCoils(byte deviceAddress, ushort startAddress, ushort coilCount, byte[] coils, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         byte byteCnt = (byte)(coilCount/8);
         if((coilCount % 8) > 0)
         {
            byteCnt++;
         }

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.WriteMultipleCoils, (short)(5 + byteCnt),
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         ModbusUtils.InsertUShort(_Buffer, dataPos, startAddress);
         ModbusUtils.InsertUShort(_Buffer, dataPos + 2, coilCount);
         _Buffer[dataPos + 4] = byteCnt;
         Array.Copy(coils, 0, _Buffer, dataPos + 5, byteCnt);

         SendReceive(deviceAddress, ModbusFunctionCode.WriteMultipleCoils, timeout, telegramLength, 4, telegramContext, ref dataPos);
      }

      /// <summary>
      /// Sends a Write Multiple Coils (0x10) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="startAddress">Address of the 1st register: 0x0000 .. 0xFFFF</param>
      /// <param name="registers">Register values to write</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual void WriteMultipleRegisters(byte deviceAddress, ushort startAddress, ushort[] registers,
                                                 int timeout = 2000)
      {
         WriteMultipleRegisters(deviceAddress, startAddress, registers, 0, registers.Length, timeout);
      }

      /// <summary>
      /// Sends a Write Multiple Coils (0x10) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="startAddress">Address of the 1st register: 0x0000 .. 0xFFFF</param>
      /// <param name="registers">Register values to write</param>
      /// <param name="offset">Offset in register of the 1st register to write</param>
      /// <param name="count">Number of registers to write</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual void WriteMultipleRegisters(byte deviceAddress, ushort startAddress, ushort[] registers, int offset, int count, int timeout = 2000)
      {
         if (offset < 0 || offset >= registers.Length)
         {
            throw new ArgumentException("offset");
         }
         if (offset + count > registers.Length)
         {
            throw new ArgumentException("count");
         }
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.WriteMultipleRegisters, (short)(5 + 2 * count),
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         ModbusUtils.InsertUShort(_Buffer, dataPos, startAddress);
         ModbusUtils.InsertUShort(_Buffer, dataPos + 2, (ushort)count);
         _Buffer[dataPos + 4] = (byte)(2 * count);
         for (int i = 0; i < count; i++)
         {
            ModbusUtils.InsertUShort(_Buffer, dataPos + 5 + 2 * i, registers[offset + i]);
         }

         SendReceive(deviceAddress, ModbusFunctionCode.WriteMultipleRegisters, timeout, telegramLength, 4, telegramContext, ref dataPos);
      }

      /// <summary>
      /// Sends a Read Write Multiple Coils (0x17) telegram to a device and waits for the response
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="writeStartAddress">Address of the 1st write register: 0x0000 .. 0xFFFF</param>
      /// <param name="writeRegisters">Register values to write</param>
      /// <param name="readStartAddress">Address of the 1st read register: 0x0000 .. 0xFFFF</param>
      /// <param name="readCount">Number of registers to read</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <returns>Returns a unsigned short array with the register values.</returns>
      /// <remarks>
      /// The write operation is performed before the read operation.
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual ushort[] ReadWriteMultipleRegisters(byte deviceAddress, ushort writeStartAddress, ushort[] writeRegisters, ushort readStartAddress, ushort readCount, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.ReadWriteMultipleRegisters, (short)(9 + 2 * writeRegisters.Length),
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         ModbusUtils.InsertUShort(_Buffer, dataPos, readStartAddress);
         ModbusUtils.InsertUShort(_Buffer, dataPos + 2, readCount);
         ModbusUtils.InsertUShort(_Buffer, dataPos + 4, writeStartAddress);
         ModbusUtils.InsertUShort(_Buffer, dataPos + 6, (ushort)writeRegisters.Length);
         _Buffer[dataPos + 8] = (byte)(2 * writeRegisters.Length);
         for (int i = 0; i < writeRegisters.Length; i++)
         {
            ModbusUtils.InsertUShort(_Buffer, dataPos + 9 + 2 * i, writeRegisters[i]);
         }

         SendReceive(deviceAddress, ModbusFunctionCode.ReadWriteMultipleRegisters, timeout, telegramLength, (short)(1 + 2 * readCount), telegramContext, ref dataPos);
         if (deviceAddress == ModbusConst.BroadcastAddress)
         {
            return null;
         }
         var readRegisters = new ushort[readCount];
         for (int n = 0; n < readRegisters.Length; ++n)
         {
            readRegisters[n] = ModbusUtils.ExtractUShort(_Buffer, dataPos + 1 + 2 * n);
         }
         return readRegisters;
      }

      /// <summary>
      /// Reads the Device Information by using the function code (0x2b / 0x0e). The methods sends as many messages as needed to read the whole data.
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="deviceIdCode">Device id code to read.</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <returns>Returns an array with all device information.</returns>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual DeviceIdentification[] ReadDeviceIdentification(byte deviceAddress,
                                                                     ModbusConformityLevel deviceIdCode,
                                                                     int timeout = 2000)
      {
         bool moreFollows = true;
         var objectId = ModbusObjectId.VendorName;
         var values = new ArrayList();
         while (moreFollows)
         {
            var v = ReadDeviceIdentification(deviceAddress, deviceIdCode, ref objectId, out moreFollows, timeout);
            for (int i = 0; i < v.Length; i++)
            {
               values.Add(v[i]);
            }
         }
         return (DeviceIdentification[]) values.ToArray(typeof (DeviceIdentification));
      }

      /// <summary>
      /// Reads the Device Information by using the function code (0x2b / 0x0e)
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="deviceIdCode">Device id code to read.</param>
      /// <param name="objectId">Object id to start at. Receives the next object id to read from if moreFollows is set to true.</param>
      /// <param name="moreFollows">Receives true if there is more information to read.</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <returns>Returns an array with all device information.</returns>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual DeviceIdentification[] ReadDeviceIdentification(byte deviceAddress, ModbusConformityLevel deviceIdCode, ref ModbusObjectId objectId, out bool moreFollows, int timeout = 2000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.ReadDeviceIdentification, 3,
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         _Buffer[dataPos] = 0x0e;
         _Buffer[dataPos + 1] = (byte)deviceIdCode;
         _Buffer[dataPos + 2] = (byte)objectId;

         SendReceive(deviceAddress, ModbusFunctionCode.ReadDeviceIdentification, timeout, telegramLength, -1, telegramContext, ref dataPos);
         if (deviceAddress == ModbusConst.BroadcastAddress)
         {
            moreFollows = false;
            return null;
         }
         moreFollows = _Buffer[dataPos + 3] != 0;
         objectId = (ModbusObjectId) _Buffer[dataPos + 4];
         byte cnt = _Buffer[dataPos + 5];

         var values = new DeviceIdentification[cnt];
         dataPos += 6;
         for (int i = 0; i < cnt; i++)
         {
            var v = new char[_Buffer[dataPos + 1]];
            for (int n = 0; n < v.Length; ++n)
            {
               v[n] = (char) _Buffer[dataPos + 2 + n];
            }
            values[i] = new DeviceIdentification
               {
                  ObjectId = (ModbusObjectId) _Buffer[dataPos],
                  Value = new string(v)
               };
            dataPos += (short)(2 + v.Length);
         }
         return values;
      }

      /// <summary>
      /// Reads one specific Device Information by using the function code (0x2b / 0x0e)
      /// </summary>
      /// <param name="deviceAddress">Address of the modbus device.</param>
      /// <param name="objectId">Object id to read.</param>
      /// <param name="timeout">Timeout for response in milli seconds.</param>
      /// <returns>Returns the requested device information.</returns>
      /// <remarks>
      /// Look at http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf for more details.
      /// </remarks>
      public virtual DeviceIdentification ReadSpecificDeviceIdentification(byte deviceAddress, ModbusObjectId objectId, int timeout = 4000)
      {
         short telegramLength;
         short dataPos;
         object telegramContext = null;

         _Interface.CreateTelegram(deviceAddress, (byte)ModbusFunctionCode.ReadDeviceIdentification, 3,
            _Buffer, out telegramLength, out dataPos, false, ref telegramContext);

         _Buffer[dataPos] = 0x0e;
         _Buffer[dataPos + 1] = 0x04;
         _Buffer[dataPos + 2] = (byte)objectId;

         SendReceive(deviceAddress, ModbusFunctionCode.ReadDeviceIdentification, timeout, telegramLength, -1, telegramContext, ref dataPos);
         if (deviceAddress == ModbusConst.BroadcastAddress)
         {
            return new DeviceIdentification();
         }
         byte cnt = _Buffer[dataPos + 5];
         if (cnt == 0)
         {
            return new DeviceIdentification();
         }

         dataPos += 6;
         var v = new char[_Buffer[dataPos + 1]];
         for (int n = 0; n < v.Length; ++n)
         {
            v[n] = (char)_Buffer[dataPos + 2 + n];
         }
         return new DeviceIdentification
            {
            ObjectId = (ModbusObjectId)_Buffer[dataPos],
            Value = new string(v)
         };
      }
   }
}
// ReSharper restore TooWideLocalVariableScope
