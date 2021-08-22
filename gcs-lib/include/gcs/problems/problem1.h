#ifndef PROBLEMS_PROBLEM1_H_
#define PROBLEMS_PROBLEM1_H_

//#include <memory>
//#include <string>

#include "problem.h"

// template<typename ... Args>
// static std::string string_format( const std::string& format, Args ... args )
//{
//    size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; //
//    Extra space for '\0' std::unique_ptr<char[]> buf( new char[ size ] );
//    snprintf( buf.get(), size, format.c_str(), args ... );
//    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want
//    the '\0' inside
//}

namespace problems {

Problem problem1();

}  // namespace problems

#endif  // PROBLEMS_PROBLEM1_H_
