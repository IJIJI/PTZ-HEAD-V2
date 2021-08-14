enum ProtocolCommands {
  joyUpdate,
  focus,
  goToCoords,
  writePos,
  writePosCoords,
  callPos,
  setCoords,
  callError,
  setLimit
};

//? joyUpdate
// formatting: identifier, camNum, joyX, joyY, joyZ, checkSum
// ID identifies the call type, in this case a joy update.
// camNum is the number of the camera being controlled.
// joyX/Y/Z are 0-255, with 127 being centered.
// Checksum is the average of all previous commands

//? focus
//! WIP

//? goToCoords
//! WIP
// formatting: identifier, camNum, xPosA, xPosB, yPosA, yPosB, zPosA, zPosB, checkSum
// ID identifies the call type.
// camNum is the number of the camera being controlled.
// X/Y/Zpos are byte values combined over A and B values. 1230 would be: 
//  - Avar: 00000100
//  - Bvar: 11001110
// checkSum is the average of all previous commands

//? writePos
// formatting: identifier, camNum, posNumber, checkCum
// ID identifies the call type.
// camNum is the number of the camera being controlled.
// posNumber is the position number to write.
// checkSum is the average of all previous commands


//? writePosCoords
//! WIP
// formatting: identifier, camNum, posNumber, xPosA, xPosB, yPosA, yPosB, zPosA, zPosB, checkSum
// ID identifies the call type.
// camNum is the number of the camera being controlled.
// X/Y/Zpos are byte values combined over A and B values. 1230 would be: 
//  - Avar: 00000100
//  - Bvar: 11001110
// posNumber is the position number to write.
// checkSum is the average of all previous commands

//? callPos
// formatting: identifier, camNum, posNumber, checkSum
// ID identifies the call type.
// camNum is the number of the camera being controlled.
// posNumber is the position number to write.
// checkSum is the average of all previous commands

//? setCoords
// formatting: identifier, camNum, xPosA, xPosB, yPosA, yPosB, zPosA, zPosB, checkSum
// Sets te coordinates of the head to the ones called.
// ID identifies the call type.
// camNum is the number of the camera being controlled.
// X/Y/Zpos are byte values combined over A and B values. 1230 would be: 
//  - Avar: 00000100
//  - Bvar: 11001110
// posNumber is the position number to write.
// checkSum is the average of all previous commands

//? callError
// formatting: identifier, camNum, errorNum, checkSum
// ID identifies the call type.
// camNum is the number of the camera being controlled.
// errorNum is the error to call.
// checkSum is the average of all previous commands

//? setLimit
// formatting: identifier, camNum, limitType, checkSum
// ID identifies the call type.
// camNum is the number of the camera being controlled.
// limitType is the type of limit to set. see enum below:

enum limitType {
  xMin,
  xMax,
  yMin,
  yMax
};

limitType limitTypes;

// checkSum is the average of all previous commands