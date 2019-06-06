// tfarg_service.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <map>
#include <set>
#include <string>

using namespace std;

#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/crc.hpp>
#include <boost/cstdint.hpp>
#include <cpprest/http_listener.h>
#include <cpprest/json.h>
#include <cpprest/base_uri.h>

#include "MyRobot_Protocol.h"
#include "ROBOTIQ3finger_function.h"

using namespace web;
using namespace web::http;
using namespace web::http::experimental::listener;

#define SERVICE_URL L"http://localhost:5002/v1/tfg/"

#define TRACE(msg)            wcout << msg
#define TRACE_ACTION(a, k, v) wcout << a << L" (" << k << L", " << v << L")\n"
#define PORT_NUM 4
#define PORT "COM"+std::to_string(static_cast<long long>(PORT_NUM))

ROBOTIQ3finger_function myROBOTIQ3finger_function(PORT);
bool _3finger_enabled = false;

void waitUntilFinishConfiguring() {
	int GACT, GMOD, GGTO, GIMC, GSTA;
	do {
		myROBOTIQ3finger_function.ROBOTIQ3finger_gripper_status_read(GACT, GMOD, GGTO, GIMC, GSTA);
	} while (GIMC < 3); // While GIMC==1 or 2, it means that the gripper is activating; When GIMC=3, activation is finished
}

void waitUntilFinishMoving() {
	int GACT, GMOD, GGTO, GIMC, GSTA;
	do {
		myROBOTIQ3finger_function.ROBOTIQ3finger_gripper_status_read(GACT, GMOD, GGTO, GIMC, GSTA);
	} while (GSTA == 0); // While GSTA==0, it means that the gripper is moving; When GSTA=3, activation is finished
}

void display_json(
	json::value const & jvalue,
	utility::string_t const & prefix)
{
	wcout << prefix << jvalue.serialize() << endl;
}

void handle_get(http_request request)
{
	TRACE(L"\nhandle GET\n");

	auto paths = http::uri::split_path(http::uri::decode(request.relative_uri().path()));

	switch (paths.size())
	{
	case 1:
		if (paths[0] == L"state")
		{
			request.reply(status_codes::NotImplemented);
		}
		else if (paths[0] == L"mode")
		{
			request.reply(status_codes::NotImplemented);
		}
		else if (paths[0] == L"position")
		{
			request.reply(status_codes::NotImplemented);
		}
		else
		{
			request.reply(status_codes::NotFound);
		}
		break;

	default:
		request.reply(status_codes::NotFound);
		break;
	}
}

void handle_request(
	http_request request,
	function<void(json::value const &, json::value &)> action)
{
	auto answer = json::value::object();

	request
		.extract_json()
		.then([&answer, &action](pplx::task<json::value> task) {
		try
		{
			auto const & jvalue = task.get();
			display_json(jvalue, L"R: ");

			if (!jvalue.is_null())
			{
				action(jvalue, answer);
			}
		}
		catch (http_exception const & e)
		{
			wcout << e.what() << endl;
		}
	})
		.wait();


	display_json(answer, L"S: ");

	request.reply(status_codes::OK, answer);
}

void handle_post(http_request request)
{
	TRACE("\nhandle POST\n");

	auto paths = http::uri::split_path(http::uri::decode(request.relative_uri().path()));

	switch (paths.size())
	{
	case 1:
		if (paths[0] == L"state")
		{
			request.reply(status_codes::NotImplemented);
		}
		else if (paths[0] == L"mode")
		{
			request.reply(status_codes::NotImplemented);
		}
		else if (paths[0] == L"position")
		{
			request.reply(status_codes::NotImplemented);
		}
		else
		{
			request.reply(status_codes::NotFound);
		}
		break;

	default:
		request.reply(status_codes::NotFound);
		break;
	}
}

void handle_put(http_request request)
{
	TRACE("\nhandle PUT\n");

	auto paths = http::uri::split_path(http::uri::decode(request.relative_uri().path()));
	switch (paths.size())
	{
	case 1:
		if (paths[0] == L"state")
		{
			handle_request(
				request,
				[](json::value const & jvalue, json::value & answer)
			{
				//* Retrieve request
				bool request_state = jvalue.as_bool();
				if (request_state)
				{
					double pos = 0;
					double speed = 110;
					double force = 15;
					myROBOTIQ3finger_function.ROBOTIQ3finger_active_on(0);
					myROBOTIQ3finger_function.ROBOTIQ3finger_set_all(pos, speed, force);
					waitUntilFinishConfiguring();

					TRACE_ACTION(L"enabled", L"state", L"");
					answer[L"state"] = json::value::string(L"<updated>");
				}
				else
				{
					myROBOTIQ3finger_function.ROBOTIQ3finger_active_off();

					TRACE_ACTION(L"disabled", L"state", L"");
					answer[L"state"] = json::value::string(L"<updated>");
				}
			});
		}
		else if (paths[0] == L"mode")
		{
			handle_request(
				request,
				[](json::value const & jvalue, json::value & answer)
			{
				//* Retrieve request
				bool request_mode = jvalue.as_integer();

				myROBOTIQ3finger_function.ROBOTIQ3finger_active_on(request_mode);
				waitUntilFinishConfiguring();
			});
		}
		else if (paths[0] == L"position")
		{
			//request.reply(status_codes::NotImplemented);
			handle_request(
				request,
				[](json::value const & jvalue, json::value & answer)
			{
				auto jobject = jvalue.as_object();
				auto jpos = jobject.find(L"pos");
				auto jspeed = jobject.find(L"speed");
				auto jforce = jobject.find(L"force");

				bool condition = (jpos != jobject.end()) && (jspeed != jobject.end()) && (jforce != jobject.end());

				if (condition)
				{
					double pos = (*jpos).second.as_double(), speed = (*jspeed).second.as_double(), force = (*jforce).second.as_double();
					myROBOTIQ3finger_function.ROBOTIQ3finger_set_all(pos, speed, force);
					myROBOTIQ3finger_function.ROBOTIQ3finger_auto_centering();
					waitUntilFinishMoving();

					TRACE_ACTION(L"executed", L"moving", L"successfully");
					answer[L"moving"] = json::value::string(L"<successfully>");
				}
			});
		}
		else
		{
			request.reply(status_codes::NotFound);
		}
		break;

	default:
		request.reply(status_codes::NotFound);
		break;
	}
}

void handle_del(http_request request)
{
	TRACE("\nhandle DEL\n");

	auto paths = http::uri::split_path(http::uri::decode(request.relative_uri().path()));

	switch (paths.size())
	{
	case 1:
		if (paths[0] == L"state")
		{
			request.reply(status_codes::NotImplemented);
		}
		else
		{
			request.reply(status_codes::NotFound);
		}
		break;

	default:
		request.reply(status_codes::NotFound);
		break;
	}
}

int main()
{
	std::cout << "Hello World! \nThis is server.";

	/**================================================== *
	 * ==========  Service  ========== *
	 * ================================================== */
	http_listener listener(SERVICE_URL);

	listener.support(methods::GET, handle_get);
	listener.support(methods::POST, handle_post);
	listener.support(methods::PUT, handle_put);
	listener.support(methods::DEL, handle_del);

	try
	{
		listener
			.open()
			.then([&listener]() {TRACE(L"\nstarting to listen\n"); })
			.wait();

		while (true);
	}
	catch (exception const & e)
	{
		wcout << e.what() << endl;
	}

	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
