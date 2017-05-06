/*
 * wrapper.h
 *
 *  Created on: 7th May 2017
 *      Author: Samuel Dudley
 */
#ifndef WRAPPER_H
#define WRAPPER_H

#include <boost/program_options.hpp>
#include <string>
#include <vector>
#include <boost/thread/thread.hpp>
#include <algorithm>
#include <ostream>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/python.hpp>

// CMAVNode headers
#include "mlink.h"
#include "asyncsocket.h"
#include "serial.h"
#include "exception.h"
#include "shell.h"
#include "configfile.h"

//Periodic function timings
#define MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS 500

class Wrapper
{
	public:
		Wrapper(std::string strConfigFile); // wrapper constructor
		void shutdown();
		void configureLinks(std::string linkConfigFile);
		void configureLogger(std::string logConfigFile);
		void runMainLoop(bool &exitMainLoop);
		void mainLoop();
		void printLinkStats();
		void shell(std::string line);

		void exitGracefully(int signum);

		static void static_exitGracefully(int signum)
		{
			instance.exitGracefully(signum);
		}

	public:
		int ret;
		boost::thread* t;
		std::vector<std::shared_ptr<mlink> > links; // holds all links
		bool verbose;
		bool exitMainLoop;

	private:
	    static Wrapper instance;


};

#endif // WRAPPER_H
