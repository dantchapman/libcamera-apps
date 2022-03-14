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
/* for time functions */
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>


#include "core/libcamera_encoder.hpp"
#include "output/output.hpp"
#include "image/image.hpp"
#include "core/still_options.hpp"

using namespace std::placeholders;
using libcamera::Stream;


struct MotionDetectOptions : public VideoOptions
{
	MotionDetectOptions() : VideoOptions()
	{
		using namespace boost::program_options;
		options_.add_options()
			("minframes", value<unsigned int>(&minframes)->default_value(50), "Minimum number of frames for a capture")
			("gap", value<unsigned int>(&gap)->default_value(20), "Smallest gap between captures in frames")
			("savedir", value<std::string>(&savedir), "Directory to save files")
			;
	}

	unsigned int minframes;
	unsigned int gap;
	std::string savedir;


	virtual void Print() const override
	{
		VideoOptions::Print();
		std::cerr << "    minframes: " << minframes << std::endl;
		std::cerr << "    gap: " << gap << std::endl;
		std::cerr << "    savedir: " << savedir << std::endl;
	}
};




class LibcameraMotionDetectApp : public LibcameraApp
{
public:
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;

	LibcameraMotionDetectApp() : LibcameraApp(std::make_unique<MotionDetectOptions>()) {}

	void StartEncoder()
	{
		createEncoder();
		encoder_->SetInputDoneCallback(std::bind(&LibcameraMotionDetectApp::encodeBufferDone, this, std::placeholders::_1));
		encoder_->SetOutputReadyCallback(encode_output_ready_callback_);
	}
	// This is callback when the encoder gives you the encoded output data.
	void SetEncodeOutputReadyCallback(EncodeOutputReadyCallback callback) { encode_output_ready_callback_ = callback; }
	void EncodeBuffer(CompletedRequestPtr &completed_request, Stream *stream)
	{
		assert(encoder_);
		StreamInfo info = GetStreamInfo(stream);
		FrameBuffer *buffer = completed_request->buffers[stream];
		libcamera::Span span = Mmap(buffer)[0];
		void *mem = span.data();
		if (!buffer || !mem)
			throw std::runtime_error("no buffer to encode");
		int64_t timestamp_ns = buffer->metadata().timestamp;
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			encode_buffer_queue_.push(completed_request); // creates a new reference
		}
		encoder_->EncodeBuffer(buffer->planes()[0].fd.get(), span.size(), mem, info, timestamp_ns / 1000);
	}
	MotionDetectOptions *GetOptions() const { return static_cast<MotionDetectOptions *>(options_.get()); }
	void StopEncoder() { encoder_.reset(); }

protected:
	virtual void createEncoder()
	{
		StreamInfo info;
		VideoStream(&info);
		if (!info.width || !info.height || !info.stride)
			throw std::runtime_error("video steam is not configured");
		encoder_ = std::unique_ptr<Encoder>(Encoder::Create(GetOptions(), info));
	}
	std::unique_ptr<Encoder> encoder_;

private:
	void encodeBufferDone(void *mem)
	{
		// If non-NULL, mem would indicate which buffer has been completed, but
		// currently we're just assuming everything is done in order. (We could
		// handle this by replacing the queue with a vector of <mem, completed_request>
		// pairs.)
		assert(mem == nullptr);
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			if (encode_buffer_queue_.empty())
				throw std::runtime_error("no buffer available to return");
			encode_buffer_queue_.pop(); // drop shared_ptr reference
		}
	}

	std::queue<CompletedRequestPtr> encode_buffer_queue_;
	std::mutex encode_buffer_queue_mutex_;
	EncodeOutputReadyCallback encode_output_ready_callback_;
};

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
static void event_loop(LibcameraMotionDetectApp &app)
{
	MotionDetectOptions  *options = app.GetOptions();
	std::string origoutput = options->output;

	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	// std::unique_ptr<Output> output2 = std::unique_ptr<Output>(Output::Create(options2));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));

	app.OpenCamera();
	app.ConfigureVideo(get_colourspace_flags(options->codec));
	app.StartEncoder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	bool motion_detected = false;
	// Monitoring for keypresses and signals.
	signal(SIGUSR1, default_signal_handler);
	signal(SIGUSR2, default_signal_handler);
	pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };
	int minframes = options->minframes;
	int frame_count = 0;
	int movie_num = 0;
	int frame_num = 0;
	int gap = options->gap;
	int frames [10000] = { }; //cant dynamically allocate in cpp rubbish
	bool motion_test = false;


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
			bool motion_val = false;
			// bool motion_val2 = false;
			completed_request->post_process_metadata.Get("motion_detect.result", motion_val);
			frames[frame_num] = motion_val;
			

			if (frame_num == gap-1)
			{
				int n = gap, sum = 0;
				for(int i = 0; i<n ; i++)
				{
					sum+=frames[i];
				}
				//std::cerr << "frame sum " << sum << std::endl;
				frame_num = 0;
				if (sum > 0) 
				{
					motion_test = true;

				}
				else {
					motion_test = false;
				}
			}
			frame_num++;

			// if (motion_val) {
			// 	motion_val2 = true;
			// }
			//std::cerr <<"frame: "<< current_capture_frame - last_capture_frame << std::endl;
			//std::cerr <<"motion: "<< motion_val << std::endl;
			// TODO - only check every few frames!!
			bool capture = true;
			if (motion_test && !motion_detected)
			{
				movie_num++;
				capture = false;
				frame_count = minframes;
				motion_detected = true;
				std::cerr <<"motion detected boyo" << std::endl;
				app.StopCamera();
				app.Teardown();
				// Get the current time
				auto t = std::time(nullptr);
                auto tm = *std::localtime(&t);
				std::ostringstream oss;
    			oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    			auto datentime = oss.str();
				std::string filename = options->savedir;
				filename.append("/").append(datentime).append("-motionmov.h264");
				options->output = filename;
				output = std::unique_ptr<Output>(Output::Create(options));
				app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
				app.ConfigureVideo(get_colourspace_flags(options->codec));
				app.StartEncoder();
				app.StartCamera();

			}
			else if (!motion_test && motion_detected && frame_count ==0)
			{
				capture = false;
				motion_detected = false;
				std::cerr <<"motion ended boyo" << std::endl;
				app.StopCamera();
				app.Teardown();
				options->output = origoutput;
				output = std::unique_ptr<Output>(Output::Create(options));
				app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
				app.ConfigureVideo(get_colourspace_flags(options->codec));
				app.StartEncoder();
				app.StartCamera();

			}
			else if (capture){
			app.EncodeBuffer(completed_request, app.VideoStream());
			app.ShowPreview(completed_request, app.VideoStream());
			
			}
			if (frame_count > 0)
			{
				frame_count--;
			}
	}
}

int main(int argc, char *argv[])
{
	std::cerr << "hello there!" << "\n";
	try
	{
		LibcameraMotionDetectApp app;
		MotionDetectOptions *options = app.GetOptions();
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
