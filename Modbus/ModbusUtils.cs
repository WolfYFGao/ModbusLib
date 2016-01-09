#if NETMF
using Microsoft.SPOT;
#endif
using System;

namespace Osre.Modbus
{
   /// <summary>
   /// Class with modbus utility functions
   /// </summary>
   public static class ModbusUtils
   {
      /// <summary>
      /// Inserts a unsigned short value into a byte array in big-endian format.
      /// </summary>
      /// <param name="buffer">Byte array to write to</param>
      /// <param name="pos">Index to write to.</param>
      /// <param name="value">Value to write.</param>
      public static void InsertUShort(byte[] buffer, int pos, ushort value)
      {
         buffer[pos] = (byte)((value & 0xff00) >> 8);
         buffer[pos + 1] = (byte)(value & 0x00ff);
      }

      /// <summary>
      /// Extracts a unsigned short value from an byte array in big-endian format.
      /// </summary>
      /// <param name="buffer">Byte array to read from</param>
      /// <param name="pos">Index to read from</param>
      /// <returns>Returns the unsigned short value.</returns>
      public static ushort ExtractUShort(byte[] buffer, int pos)
      {
         return (ushort)((buffer[pos] << 8) + buffer[pos + 1]);
      }
      
       /// <summary>
      /// Convert Byte to Ascii
      /// </summary>
      /// <param name="bite">Byte to convert to Ascii.</param>
      /// <returns>Returns the 8 bit LRC</returns>
      public static void InsertByteAsAscii(byte[] buffer, int pos, byte value)
      {
          byte to_ascii = (byte)((value & 0xf0) >> 4);
          buffer[pos] = (byte)(to_ascii < 10 ? to_ascii + 0x30 : to_ascii + 0x41 - 10);
          to_ascii = (byte)(value & 0x0f);
          buffer[pos + 1] = (byte)(to_ascii < 10 ? to_ascii + 0x30 : to_ascii + 0x41 - 10);
      }

      /// <summary>
      /// Extracts a unsigned short value from an byte array in big-endian format.
      /// </summary>
      /// <param name="buffer">Byte array to read from</param>
      /// <param name="pos">Index to read from</param>
      /// <returns>Returns the unsigned short value.</returns>
      public static byte ExtractAsciiAsByte(byte[] buffer, int pos)
      {
          byte value_extracted = 0;

          if (isAscii(buffer[pos]))
          {
              if ((buffer[pos] >= 0x30) && (buffer[pos] <= 0x39))
              {
                  value_extracted = (byte)(buffer[pos] - 0x30);
              }
              else if ((buffer[pos] >= 0x41) && (buffer[pos] <= 0x46))
              {
                  value_extracted = (byte)(buffer[pos] - 0x41 + 10);
              }
              else if ((buffer[pos] >= 0x61) && (buffer[pos] <= 0x66))
              {
                  value_extracted = (byte)(buffer[pos] - 0x61 + 10);
              }
              else
              {
#if NETMF
                  Debug.Print("Error Extracting Ascii: " + buffer[pos].ToString() + " at pos " + pos.ToString());
#endif
                  throw new ModbusException(ModbusErrorCode.IllegalDataValue);
              }
          }
          else
          {
#if NETMF
              Debug.Print("Error Extracting Ascii: " + buffer[pos].ToString() + " at pos " + pos.ToString());
#endif
              throw new ModbusException(ModbusErrorCode.IllegalDataValue);
          }
          value_extracted <<= 4;
          if (isAscii(buffer[pos + 1]))
          {
              if ((buffer[pos + 1] >= 0x30) && (buffer[pos + 1] <= 0x39))
              {
                  value_extracted |= (byte)(buffer[pos + 1] - 0x30);
              }
              else if ((buffer[pos + 1] >= 0x41) && (buffer[pos + 1] <= 0x46))
              {
                  value_extracted |= (byte)(buffer[pos + 1] - 0x41 + 10);
              }
              else if ((buffer[pos + 1] >= 0x61) && (buffer[pos + 1] <= 0x66))
              {
                  value_extracted |= (byte)(buffer[pos + 1] - 0x61 + 10);
              }
              else
              {
#if NETMF
                  Debug.Print("Error Extracting Ascii: " + buffer[pos + 1].ToString() + " at pos " + (pos + 1).ToString());
#endif
                  throw new ModbusException(ModbusErrorCode.IllegalDataValue);
              }
          }
          else
          {
#if NETMF
              Debug.Print("Error Extracting Ascii: " + buffer[pos + 1].ToString() + " at pos " + (pos + 1).ToString());
#endif
              throw new ModbusException(ModbusErrorCode.IllegalDataValue);
          }
          return value_extracted;
      }

      /// <summary>
      /// Calculates the Modbus RTU CRC16 checksumm
      /// </summary>
      /// <param name="buffer">Buffer containing the telegram.</param>
      /// <param name="count">Count of bytes to use for CRC (not including the 2 bytes for CRC).</param>
      /// <returns>Returns the 16 bit CRC</returns>
      public static ushort CalcCrc(byte[] buffer, int count)
      {
         ushort crc = 0xffff;
         for (int i = 0; i < count; i++)
         {
            crc = (ushort)(crc ^ buffer[i]);
            for (int j = 0; j < 8; j++)
            {
               bool lsbHigh = (crc & 0x0001) != 0;
               crc = (ushort)((crc >> 1) & 0x7FFF);

               if (lsbHigh)
               {
                  crc = (ushort)(crc ^ 0xa001);
               }
            }
         }
         return crc;
      }

      /// <summary>
      /// Calculates the Modbus ASCII LRC checksum
      /// </summary>
      /// <param name="buffer">Buffer containing the telegram.</param>
      /// <param name="count">Count of bytes to use for LRC (not including the 2 bytes for LRC).</param>
      /// <returns>Returns the 8 bit LRC</returns>
      public static Byte CalcLrc(byte[] buffer, int offset, int last_index)
      {
          ushort lrc = 0;

          // LRC Calculation
          for (int i = offset; i < last_index; i++)
          {
              lrc += buffer[i];
          }
          // Take only 8 bits from lrc
          lrc &= 0x00FF;
          // Procede with XOR logic
          lrc ^= 0x00FF;
          // Sum 1
          lrc += 1;
          // Take only 8 bits
          lrc &= 0x00FF;
          return (Byte) lrc;
      }

      /// <summary>
      /// Convert Byte to Ascii
      /// </summary>
      /// <param name="bite">Byte to convert to Ascii.</param>
      /// <returns>Returns the 8 bit LRC</returns>
      public static string Hex(int bite)
      {
          string ascii = "";

          bite &= 0x000000FF;

          int hex_to_convert = ((bite & 0x00F0) >> 4);

          ascii += (hex_to_convert < 10 ? (char)(hex_to_convert + 0x30) : (char)(hex_to_convert + 0x41 - 10));

          hex_to_convert = (bite & 0x000F);

          ascii += (hex_to_convert < 10 ? (char)(hex_to_convert + 0x30) : (char)(hex_to_convert + 0x41 - 10));

          return ascii;
      }
   
       private static bool isAscii(byte value)
      {
          if ((value >= 0x30 && value <= 0x39) || (value >= 0x41 && value <= 0x46) || (value >= 0x61 && value <= 0x66))
          {
            return true;
          }

          return false;
      }
   }
}
