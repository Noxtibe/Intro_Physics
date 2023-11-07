//
//  main.cpp
//
#include "application.h"

/*
====================================================
main
====================================================
*/
int main( int argc, char * argv[] ) {
	application = new Application;
	application->Initialize();

	application->MainLoop();

	delete application;
	return 0;
}
