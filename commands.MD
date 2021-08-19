
Table of contents
=================

<!--ts-->
   * [PTZ Protocol](#ptz-protocol)
     * [Command structure](#command-structure)
       * [Rules](#rules)
       * [Cam Number](#camnum)
       * [Commands](#commands)
       * [Check Sum](#checksum)
       * [Terminator Byte](#terminator-byte)
     * [Commands Explained](#commands-explained)
       * [joyUpdate](#joyupdate-1)
       * [focus](#focus-2)
       * [goToCoords](#gotocoords-3)
       * [writePos](#writepos-4)
       * [writePosCoords](#writeposcoords-5)
       * [callPos](#callpos-6)
       * [setCoords](#setcoords-7)
       * [callError](#callerror-8)
       * [SetLimit](#setlimit-9)
     * [Command Examples](#command-examples)

<!--te-->



**PTZ Protocol**
============

## **Introduction:**

The PTZ protocol is balahaadjodnaox

</br>

### **Rules:**

* The protocol has a fixed command length of 15 bytes. Each command should end with the terminator byte, 255 (0xFF).
* 255 or 0xFF is reserved for the terminator byte. All other bytes can be 0 - 254. Any byte that is inputted as 255 should be converted to 254.
* The baud rate of the protocol is 115200.
* While doing a move using the joyUpdate command, the movement will be halted if there has passed more than 500 ms from the last received update.

</br>

# Command structure:



| Byte |        Data        |
|:----:|:------------------:|
|   0  |       camNum       |
|   1  | command identifier |
| 2-11 |        Data        |
|  12  |      checkSum      |
|  13  |   Terminator Byte  |

</br>



### **camNum:**
The camera number being adressed. Value from 0 to 254.

</br>

### **Commands:**

| Command         | camNum | Command Identifier |   Data 0  | Data 1 | Data 2 | Data 3 | Data 4 | Data 5 | Data 6 | Data 7 | Data 8 | Data 9 | Data 10 |         checkSum        | terminatorByte |         Finished         |
|-----------------|:------:|:------------------:|:---------:|:------:|:------:|:------:|:------:|:------:|:------:|:------:|:------:|:------:|:-------:|:-----------------------:|:--------------:|:------------------------:|
|_joyUpdate_      |  0-254 |          1         |    joyX   |  joyY  |  joyZ  |  joyA  |        |        |        |        |        |        |         | (camNum - Data10) % 256 |   255 (0xFF)   |    :heavy_check_mark:    |
|_focus_          |  0-254 |          2         |           |        |        |        |        |        |        |        |        |        |         | (camNum - Data10) % 256 |   255 (0xFF)   | :heavy_multiplication_x: |
|_goToCoords_     |  0-254 |          3         |   xPosA   |  xPosB |  yPosA |  yPosB |  zPosA |  zPosB |  aPosA |  aPosB |        |        |         | (camNum - Data10) % 256 |   255 (0xFF)   | :heavy_multiplication_x: |
|_writePos_       |  0-254 |          4         | posNumber |        |        |        |        |        |        |        |        |        |         | (camNum - Data10) % 256 |   255 (0xFF)   |    :heavy_check_mark:    |
|_writePosCoords_ |  0-254 |          5         | posNumber |  xPosA |  xPosB |  yPosA |  yPosB |  zPosA |  zPosB |  aPosA |  aPosB |        |         | (camNum - Data10) % 256 |   255 (0xFF)   | :heavy_multiplication_x: |
|_callPos_        |  0-254 |          6         | posNumber |        |        |        |        |        |        |        |        |        |         | (camNum - Data10) % 256 |   255 (0xFF)   |    :heavy_check_mark:    |
|_setCoords_      |  0-254 |          7         |   xPosA   |  xPosB |  yPosA |  yPosB |  zPosA |  zPosB |  aPosA |  aPosB |        |        |         | (camNum - Data10) % 256 |   255 (0xFF)   |    :heavy_check_mark:    |
|_callError_      |  0-254 |          8         |  errorNum |        |        |        |        |        |        |        |        |        |         | (camNum - Data10) % 256 |   255 (0xFF)   |    :heavy_check_mark:    |
|_setLimit_       |  0-254 |          9         | limitType |        |        |        |        |        |        |        |        |        |         | (camNum - Data10) % 256 |   255 (0xFF)   | :heavy_multiplication_x: |

</br>

### **Checksum:**
The checksum is calculated by summing byte 0-12 and calculating the modulus 256. Because the value 255 is reserved, the checksum is made 254 if it results in 255.

For example, if the sum of byte 0-12 equals 831;
```c++

int byteSum = 831;

checksum = 831 % 256; // 63

if (checksum >= 255)
  checksum = 254;

//Checksum is 63
```

### **Terminator Byte:**
The terminator byte allways is 0xFF or 255.

</br>

# **Commands Explained:**

### **joyUpdate (1):**
Moves the head. joyX/Y/Z/A, are speed values. The correct formatting is 1-254, with 127 being the middle.
This command should be repeated at least every 200ms for consistent movement. After 500ms without update the head will halt movement.

### **focus (2):**
WIP

### **goToCoords (3):**
Moves the head to coordinates. Coordinates are divided over two bytes, A and B bytes. As there is a limit of 254 per byte, there are only 7 usable bytes. The function stacks the first seven bits of both bytes to create a 14 bit value.

If bit    A = <span style="color:pink"><s>0</s>1101010</span>
and bit   B = <span style="color:lightblue"><s>0</s>0111011</span> it will combine to <span style="color:lightblue">0111011</span><span style="color:pink">1101010</span>

### **writePos (4):**
Writes posNumber to the memory of the head. Valid positions are 0-254.

### **writePosCoords (5);**
WIP

### **callPos (6):**
Makes the head move to a position. valid posNumbers are 0-254.

### **setCoords (7);**
WIP

### **callError (8);**
Calls the error of errorNum. Valid values are 0-2.

```C++
enum errorType {
  unknown,
  emergenyStop,
  collision
};
```

### **setLimit (9);**
Sets a limt. Chosen by limitType. Valid values are 0-3

```C++
enum limitType {
  xMin,
  xMax,
  yMin,
  yMax
};
```

</br>

# **Command Examples:**

Move camera 2, right:
```
0x02 0x01 0xFE 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x01 0xFF
```

Move camera 33 to pos 12:
```
0x21 0x06 0x0C 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x51 0xFF
```
