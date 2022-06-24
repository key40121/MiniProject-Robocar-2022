#include <cv.h>
#include <highgui.h>
#include "opencv2/opencv.hpp"

namespace zmp {
	namespace zrc {

struct LaneResult {
	short xpos_l[8];
	short xops_r[8];
	short ypos_l[8];
	short ypos_r[8];
};

struct CalParam {
	int	cap_num;
	int pat_row;
	int pat_col;
	int pat_size;
	int all_points;
	float chess_size;
};


enum IMG_PROC_FLG{
	PROC_ORG		= 0x01, 
	PROC_CORRECT	= 0x02,
	PROC_GRY		= 0x04,
	PROC_EDG		= 0x08,
	PROC_CAL		= 0x10,
	PROC_LANE		= 0x20,
};

class ImageReceiveHandler {
public:
	virtual void OnRcvOrgImg() = 0;
	virtual void OnRcvCorrectImg() = 0;
	virtual void OnRcvGryImg() = 0;
	virtual void OnRcvEdgImg() = 0;
	virtual void OnRcvCalImg() = 0;
	virtual void OnRcvLaneResult() = 0;
};

class ImageSensor {
public:
	ImageSensor();
	virtual ~ImageSensor();

	bool Init(int index);
	bool SetReceiveHandler(ImageReceiveHandler *handler);
	bool SetOutputImg(int flg);
	bool SetThreshold(int threshold);
	bool SetCalParam(CalParam param);
	bool ReqCalCap();

	bool Start();
	bool Stop();

	bool GetOrgImg(IplImage* org);
	bool GetCorrectImg(IplImage* correct);
	bool GetGryImg(IplImage* gry);
	bool GetEdgImg(IplImage* bin);
	bool GetLaneResult(LaneResult* lane);

private:
	bool	_b_stop_org_thread; 
	pthread_t	_receive_org_thread;
	void ReceiveThread();
	static void *ReceiveThreadEntry(void *arg);

	bool	_b_stop_correct_thread; 
	pthread_t	_proc_correct_thread;
	void UnidistThread();
	static void *UnidistThreadEntry(void *arg);

	bool	_b_stop_gry_thread; 
	pthread_t	_proc_gry_thread;
	void GryThread();
	static void *GryThreadEntry(void *arg);

	bool	_b_stop_edg_thread; 
	pthread_t	_proc_edg_thread;
	void EdgThread();
	static void *EdgThreadEntry(void *arg);

	bool	_b_stop_cal_thread; 
	pthread_t	_proc_cal_thread;
	void CalThread();
	static void *CalThreadEntry(void *arg);

	bool	_b_stop_lane_thread; 
	pthread_t	_proc_lane_thread;
	void LaneThread();
	static void *LaneThreadEntry(void *arg);

	IplImage *_orgImg;
	IplImage *_correctImg;
	IplImage *_gryImg;
	IplImage *_edgImg;
	int 	_procFlg;
	LaneResult _laneResult;	
	ImageReceiveHandler	*_handler;
	int		_index;
	struct CalParam _calP;
	bool	_req_calFlg;
	int			_binTh;
};

	}
}

