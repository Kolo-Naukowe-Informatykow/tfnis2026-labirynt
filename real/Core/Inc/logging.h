#ifndef LOGGING_H
#define LOGGING_H

#define MAX_LOG_LEN 128

typedef struct {
	char msg[MAX_LOG_LEN];
} LogMsg_t;

void Logging_Init(void);
void Logging_Print(const char* fmt, ...);

void logging_exec(void);

#endif