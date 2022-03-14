/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_vid.cpp - libcamera video record app.
 */

#include <chrono>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>

#include "core/libcamera_encoder.hpp"
#include "output/output.hpp"
#include "image/image.hpp"
#include "core/still_options.hpp"

using namespace std::placeholders;
using libcamera::Stream;


// Some keypress/signal handling.

static int signal_received;
static void default_signal_handler(int signal_number)
{
	signal_received = signal_number;
	std::cerr << "Received signal " << signal_number << std::endl;
}

static int get_key_or_signal(VideoOptions const *options, pollfd p[1])
{
	int key = 0;
	if (options->keypress)
	{
		poll(p, 1, 0);
		if (p[0].revents & POLLIN)
		{
			char *user_string = nullptr;
			size_t len;
			[[maybe_unused]] size_t r = getline(&user_string, &len, stdin);
			key = user_string[0];
		}
	}
	if (options->signal)
	{
		if (signal_received == SIGUSR1)
			key = '\n';
		else if (signal_received == SIGUSR2)
			key = 'x';
		signal_received = 0;
	}
	return key;
}

static int get_colourspace_flags(std::string const &codec)
{
	if (codec == "mjpeg" || codec == "yuv420")
		return LibcameraEncoder::FLAG_VIDEO_JPEG_COLOURSPACE;
	else
		return LibcameraEncoder::FLAG_VIDEO_NONE;
}

// The main even loop for the application.

static void event_loop(LibcameraEncoder &app)
{
	VideoOptions const *options = app.GetOptions();
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));

	app.OpenCamera();
	app.ConfigureVideo(get_colourspace_flags(options->codec));
	app.StartEncoder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();
	unsigned int last_capture_frame = 0;
	unsigned int last_capture = 0;
	// Monitoring for keypresses and signals.
	signal(SIGUSR1, default_signal_handler);
	signal(SIGUSR2, default_signal_handler);
	pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

	for (unsigned int count = 0; ; count++)
	{

		LibcameraEncoder::Msg msg = app.Wait();
		if (msg.type == LibcameraEncoder::MsgType::Quit)
			return;
		else if (msg.type != LibcameraEncoder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
		int key = get_key_or_signal(options, p);
		if (key == '\n')
			output->Signal();

		if (options->verbose)
			std::cerr << "Viewfinder frame " << count << std::endl;
		auto now = std::chrono::high_resolution_clock::now();
		bool timeout = !options->frames && options->timeout &&
					   (now - start_time > std::chrono::milliseconds(options->timeout));
		bool frameout = options->frames && count >= options->frames;
		if (timeout || frameout || key == 'x' || key == 'X')
		{
			if (timeout)
				std::cerr << "Halting: reached timeout of " << options->timeout << " milliseconds.\n";
			app.StopCamera(); // stop complains if encoder very slow to close
			app.StopEncoder();
			return;
		}
		
		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		if (app.VideoStream()) 
		{
			bool motion_val = false;
			completed_request->post_process_metadata.Get("motion_detect.result", motion_val);
			int current_capture_frame = completed_request->sequence;
			//std::cerr <<"frame: "<< current_capture_frame - last_capture_frame << std::endl;
			if ((current_capture_frame - last_capture_frame >= 50) && motion_val) 
			{
				last_capture_frame = current_capture_frame;
				std::cerr << motion_val << std::endl;
				app.StopCamera();
				app.Teardown();
				app.ConfigureStill();
				app.StartCamera();
			}
			else{
			app.EncodeBuffer(completed_request, app.VideoStream());
			app.ShowPreview(completed_request, app.VideoStream());
			}
		}
		else if (app.StillStream())
		{
			
			last_capture++;
			app.StopCamera();


			StillOptions stilloptions;
			stilloptions.quality = 93;
			stilloptions.shutter = 100000;
			stilloptions.immediate = true;
			stilloptions.gain=3;
			stilloptions.height = options->height;
			stilloptions.width = options->width;

			StreamInfo info;
			libcamera::Stream *stream = app.StillStream(&info);
			const std::vector<libcamera::Span<uint8_t>> mem = app.Mmap(completed_request->buffers[stream]);

			// Make a filename for the output and save it.
			std::string filename = "motion";
			filename.append(std::to_string(last_capture));
			filename.append(".jpeg");

			std::cerr << "Save image " << filename << std::endl;
			//LibcameraStillApp app;


			//std::cerr << "Image quality" << &stilloptions->quality << std::endl;
			jpeg_save(mem, info, completed_request->metadata, std::string(filename), app.CameraId(), &stilloptions);

			// Restart camera in preview mode.
			app.Teardown();
			app.ConfigureVideo(get_colourspace_flags(options->codec));
			app.StartEncoder();
			app.StartCamera();
		}
	}
}

int main(int argc, char *argv[])
{
	std::cout << "hello there!" << "\n";
	try
	{
		LibcameraEncoder app;
		VideoOptions *options = app.GetOptions();
		//StillOptions *stoptions = stillapp.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		std::cerr << "ERROR: *** " << e.what() << " ***" << std::endl;
		return -1;
	}
	return 0;
}
