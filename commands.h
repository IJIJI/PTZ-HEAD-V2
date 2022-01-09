



const String commandNames[] = {"null", "joyUpdate", "focus", "goToCoords", "writePos", "writePosCoords", "callPos", "setCoords", "callError", "setLimit", "home"};

const byte joyUpdateCommand       =   B00000001;
const byte focusCommand           =   B00000010;  //! WIP
const byte goToCoordsCommand      =   B00000011;  //! WIP
const byte writePosCommand        =   B00000100;
const byte writePosCoordsCommand  =   B00000101;  //! WIP
const byte callPosCommand         =   B00000110;
const byte setCoordsCommand       =   B00000111;  //! WIP
const byte callErrorCommand       =   B00001000;
const byte setLimitCommand        =   B00001001;  //! WIP
const byte homeCommand            =   B00001010;  //! WIP TODO add home types
const byte setZeroCommand         =   B00001011;  //TODO add axis types


const byte terminatorByte         =   0xFF;



//? Command length = 15 bytes

// byte 0     =   camNum
// byte 1     =   command identifier
// byte 2-12  =   data
// byte 13    =   checkSum 
// byte 14    =   terminatorByte


//? Checksum
// Sum off all bytes excluding startByte mod 256.
// checkSum = (sum of bytes)%256



enum errorType {
  unknown,
  emergenyStop,
  collision
};

errorType errorTypes;



enum limitType {
  xMin,
  xMax,
  yMin,
  yMax
};

limitType limitTypes;
