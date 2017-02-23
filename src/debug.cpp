
void debug(String message) {
    char msg [50];
    sprintf(msg, message.c_str());
    //Particle.publish("chronicle-debug", msg);
    Serial.println(message);
}
void debug(String message, int value) {
    char msg [50];
    sprintf(msg, message.c_str(), value);
    debug(msg);
}
void debug(String message, float value) {
    char msg [50];
    sprintf(msg, message.c_str(), value);
    debug(msg);
}
void debug(String message, float value, float value2) {
    char msg [50];
    sprintf(msg, message.c_str(), value, value2);
    debug(msg);
}
void debug(String message, int value, int value2) {
    char msg [50];
    sprintf(msg, message.c_str(), value, value2);
    debug(msg);
}
void debug(String message, char *value) {
    char msg [50];
    sprintf(msg, message.c_str(), value);
    debug(msg);
}
