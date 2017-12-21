#ifndef METHOD_NOT_IMPLEMENTED_EXCEPTION_HPP_
#define METHOD_NOT_IMPLEMENTED_EXCEPTION_HPP_

class METHOD_NOT_IMPLEMENTED: public std::exception
{
	virtual const char* getException() const throw() {
		return "Method not implemented";
	}
} methodNotImplementedException;

#endif // METHOD_NOT_IMPLEMENTED_EXCEPTION_HPP_
