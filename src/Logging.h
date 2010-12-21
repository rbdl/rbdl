#ifndef LOGGING_H
#define LOGGING_H

#include <sstream>

#ifndef ENABLE_LOGGING
	#define LOG if (false) LogOutput 
#else
	#define LOG LogOutput
#endif

extern std::ostringstream LogOutput;
void ClearLogOutput ();

#endif /* LOGGING_H */
