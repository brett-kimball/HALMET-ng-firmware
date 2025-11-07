#pragma once
void AISGatewayInit();          // call once from setup()
void AISGatewayLoop();          // call from loop()
void AISSendCommand(const char* cmd);   // send a $PSRT… command