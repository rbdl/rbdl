#ifndef LOGGING_H
#define LOGGING_H

#include <sstream>

class _NoLogging;

#ifndef ENABLE_LOGGING
	#define LOG if (false) LogOutput 
	#define SUPPRESS_LOGGING ;
#else
	#define LOG LogOutput
	#define SUPPRESS_LOGGING _NoLogging _nolog
#endif

extern std::ostringstream LogOutput;
void ClearLogOutput ();

class _NoLogging {
	public:
		_NoLogging() {
			log_backup.str("");
			log_backup << LogOutput.str();
		}
		~_NoLogging() {
			LogOutput.str("");
			LogOutput << log_backup.str();
		}

	private:
		std::ostringstream log_backup;
};

#endif /* LOGGING_H */
