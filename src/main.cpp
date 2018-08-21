/**
 * @file src/main.cpp
 *
 * The main file that starts the simulator interface to run in the 
 * background
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jun 2017
 */
 
#include "Interface.h"

int main()
{
	Interface* interface = new Interface();
	interface->init();
	while (true)
		interface->update();
	return 0;
}
