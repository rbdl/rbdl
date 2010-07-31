#ifndef LOGGING_H
#define LOGGING_H

#include <sstream>

#ifdef NO_LOGGING
	#define LOG if (false) LogOutput 
#else
	#define LOG LogOutput
#endif

extern std::ostringstream LogOutput;
void ClearLogOutput ();

#endif /* LOGGING_H */
