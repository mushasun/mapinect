#ifndef _MY_MONITOR
#define _MY_MONITOR

class monitor
{
public:
	static bool SetDisplayResolution(long PelsWidth, long PelsHeight);
	static void SetDisplayDefaults();
	static void SetFullScreen();
};

#endif