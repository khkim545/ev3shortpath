#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <vector>
#include <cmath>

#include <fcntl.h>
#include <unistd.h>

#include "lmstypes.h"
#include "bytecodes.h"

#define FALSE			0
#define TRUE			1

//CONTROL///////////////////////////////////////////////////////////////////////
#define EV3_DEV_NAME "/dev/hidraw0"
#define MAX_BUF_LEN		128
#define MAGIC_KEY		0xFA

#define PORT_MOTOR_L 	1	// port A
#define PORT_MOTOR_R 	2	// port B
#define SPEED_MOTOR_MAX	100
#define SPEED_MOTOR_MIN (-100)

#define SPEED_MOTOR_SAT 12.0
#define QR_SPEED_ADJ	0.85
#define SPEED_ADJ		1.80
#define ERROR_COUNT		5
#define PI 3.141592

char get_key(void) {
	char key = '\0';
	char line[MAX_BUF_LEN] = {0, };
	printf(">> ");
	scanf("%s", line);
	return line[0];
}

void print_buf(const char *heading, char *p, int len) {
	int i;
	printf("%s", heading);
	for (i = 0; i < len; i++) {
		printf(" %02x", p[i]);
		if (((i+1) % 25) == 0) printf("\n%s", heading);
	}
	//printf("\n");
	printf(" (len: %d)\n", len);
}

int get_command_length(char *p, int maxlen) {
	int i;
	//for (i = 0; i < maxlen; i++) {
	for (i = 2; i < maxlen; i++) {
		//if (p[i] == opOBJECT_ENDY)
		if (p[i] == opOBJECT_END && p[i + 1] == MAGIC_KEY)
			//return i;
			return i + 1;
	}
	return 0;
}

char direct_reply_buf[MAX_BUF_LEN] = {0, };
int EV3_send_command(int fd, char *direct_command_buf, int reply_needed) {
	int n;
	n = get_command_length(direct_command_buf, MAX_BUF_LEN);
	//direct_command_buf[1] = n / 256;
	//direct_command_buf[0] = n - direct_command_buf[1] * 256;
	direct_command_buf[1] = (n - 2) / 256;
	direct_command_buf[0] = (n - 2) - direct_command_buf[1] * 256;
	print_buf("[COMMAND]", direct_command_buf, n);
	//if(write(fd, direct_command_buf, n+1) < 0) {
	if(write(fd, direct_command_buf, n) < 0) {
		printf("write error!\n");
		return -1;
	}
	memset(direct_reply_buf, 0, sizeof(direct_reply_buf));
	if (reply_needed == FALSE)
		return 0;

	/* if reply needed */
	if((n = read(fd, direct_reply_buf, sizeof(direct_reply_buf))) < 0) {
		printf("read error!\n");
		return -1;
	}
	if (n > 0)
		n = direct_reply_buf[0] + ((int) direct_reply_buf[1]) * 256;
	print_buf("[ REPLY ]", direct_reply_buf, n + 2);
	return 1;
} 

char motor_command_buf[MAX_BUF_LEN] = {
	//0x00, 0x00, // command size
	0x3C, 0x00, // command size
	0xFF, 0xFF, // sequence counter
	0x00, 		// type of command : DIRECT_COMMAND_REPLY
	//0x01, 0x00, // message header
	0x00, 0x00, // message header

	/* byte codes */
	opOUTPUT_POWER, 
	LC0(0), 			// layer
	LC0(PORT_MOTOR_L),	// port 
	LC1(0),				// speed

	opOUTPUT_POWER, 
	LC0(0),				// layer 
	LC0(PORT_MOTOR_R), 	// port
	LC1(0),				// speed

	opOUTPUT_START, 
	LC0(0), 
	LC0(PORT_MOTOR_L + PORT_MOTOR_R),

	//opOBJECT_END
	opOBJECT_END,
	MAGIC_KEY
};

int update_motor_speed(int fd, int speed_left, int speed_right){
	int ret = TRUE;
	if(speed_left >= SPEED_MOTOR_MAX || speed_right >= SPEED_MOTOR_MAX 
		|| speed_left <= SPEED_MOTOR_MIN || speed_right <= SPEED_MOTOR_MIN)
		return FALSE;
	motor_command_buf[11] = speed_left;
	motor_command_buf[16] = speed_right;
	EV3_send_command(fd, motor_command_buf, FALSE);
	printf("motor(L,R) = (%d,%d)\n", speed_left, speed_right);
	return ret;
}

//LINE//////////////////////////////////////////////////////////////////////////
/*
 * 0: Raspberry Camera
 * 1: Others 
 */
#define CAM_NUM			0

/* 
 * if brightness > BRIGHTNESS_ADJ * brightness_average
 * is FLOOR(255), else LINE(0)
 */
#define BRIGHTNESS_ADJ	0.25

#define LINE			0
#define FLOOR			255

/* SMALL_WIDTH : SMALL_HEIGHT == 4 : 3 */
#define SMALL_WIDTH		32
#define SMALL_HEIGHT	24

/* CAP_WIDTH : CAP_HEIGHT == 4 : 3 */
#define CAP_WIDTH		640	// px
#define CAP_HEIGHT		480	// px

#define CAP_CONSTRAST	0.7
#define CAP_FPS			2.95	// Frames per Second
#define WAIT_KEY		27	// ESC key

#define NODE_NUM 		6
#define MAXIMUM			255
#define INFINITE		10000
#define START_NODE		0
#define END_NODE		4
#define START_ANGLE		-115
#define BACK_SPEED_RIGHT 0	
#define BACK_SPEED_LEFT	 47.5

/* if CAL_MOMDE, caluate degrees when meet QRcode
   if not, just get degrees from QRcode information */
#define CAL_MODE		1


using namespace cv;
using namespace std;
using namespace zbar;

enum Direction{
	TOP = 0,
	BOTTOM,
	LEFT, 
	RIGHT,
	END,
	HALF,
	QUATER
};
class Pattern{
public:
	int size(){ return this->n; }
	uchar *data(){ return this->array; }
	Direction direction(){ return this->dir; }

	Point* toPoint(int p){
		Point *ret = NULL;
		if(p < 0 || p >= this->n) return NULL;

		switch(this->dir){
		case TOP: 		ret = new Point(p, 0); break;
		case BOTTOM: 	ret = new Point(p, this->n / 4 * 3 - 1); break;
		case LEFT: 		ret = new Point(0, p); break;
		case RIGHT: 	ret = new Point(this->n / 3 * 4 - 1, p); break;
		case HALF:	ret = new Point(p, this->n / 4 * 3 - 12); break;	
//
		case QUATER:	ret = new Point(p, this->n / 4 * 3 - 6); break;	
		default: 		ret = NULL; break;
		}
		//cout << "ret:" << (void*) ret << endl;
		return ret;
	}

	void set(uchar *array, int n, Direction dir){
		this->n = n;
		if(this->array != NULL) delete this->array;
		this->array = new uchar[n];
		for(int i = 0; i < n; i++) this->array[i] = array[i];
		this->dir = dir;
	}

	Pattern(){
		this->n = 0;
		this->array = NULL;
		this->dir = END;
	}
	Pattern(uchar* array, int n, Direction dir){
		this->n = 0;
		this->array = NULL;
		this->dir = END;
		this->set(array, n, dir);
	}
	Pattern(Pattern *p){
		this->n = 0;
		this->array = NULL;
		this->dir = END;
		this->set(p->data(), p->size(), p->direction());
	}
	~Pattern(){
		if(this->array != NULL) delete this->array;
	}

private:
	int n;
	uchar *array;
	Direction dir;
};

void printData(Mat m){
	int width = m.cols;
	int height = m.rows;
	uchar *data = (uchar *)m.data;
	
	cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< 3.4 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
	cout << "---------------------------------------------------------------" << endl;
	cout << "w: " << width << ", " << "h: " << height << endl;
	for(int y = 0; y < height; y+=1){
		for(int x = 0; x < width; x+=1){
			int temp = data[y * width + x];
			char output;

			if(temp < 50) output = '#';
			else if(temp < 100) output = '+';
			else if(temp < 150) output = '-';
			else if(temp < 200) output = '.';
			else output = ' ';

			cout << output << " ";
		}
		cout << endl;
	}
	cout << "---------------------------------------------------------------" << endl;
	return;
}

Mat quantizeMat(Mat m, double adj = BRIGHTNESS_ADJ){
	int width = m.cols;
	int height = m.rows;
	Mat ret = m;
	uchar *data = (uchar *)ret.data;

	/*  get brightness average */
	int sum = 0;
	double avg = 0.0;
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			sum += data[i * width + j];
		}
	}
	avg = (double)sum / (height * width);
	cout << "average: " << avg << endl;

	/* get pixel which has brightness above average */
	double Avg = avg * adj;
	cout << "adj average: " << Avg << endl;
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
//			printf("%3d ", data[i * width + j]);
			if(data[i * width + j] > Avg){
				data[i * width + j] = FLOOR;
			}
			else
				data[i * width + j] = LINE;
		}
//		cout<<endl;
	}

	return ret;
}
int findLongestPattern(Pattern *p, int *end){
	int ret = 0;
	uchar *arr = p->data();
	int n = p->size();

	int maxStart = -1;
	int maxLen = -1;
	int maxEnd = -1; 
	for(int i = 0; i < n; i++){
		/* start */
		if(maxStart == -1 && arr[i] == LINE) maxStart = i;
		/* end */
		else if(maxStart != -1 && (arr[i] == FLOOR || i == n - 1)){
			int len = i - maxStart;
			/* update maxLen */
			if(maxLen <= len){
				maxLen = len;
				maxEnd = i;
				
				/* return values */
				ret = maxLen;
				*end = maxEnd;
			}			
			/* ready to restart */
			maxStart = -1;
		}
		/* now is searching... */
		else if(maxStart == -1 && arr[i] == FLOOR) continue;
		/* now is measuring... */
		else if(maxStart != -1 && arr[i] == LINE) continue;
		
	}
	return ret;
}
int findLongestPattern2(Pattern *p, int *end, int max, int endCheck){
	int ret = 0;
	uchar *arr = p->data();
	int n = p->size();

	int maxStart = -1;
	int maxLen = 1;
	int maxEnd = -1; 
	for(int i = 0; i < n; i++){
		/* start */
		if(maxStart == -1 && arr[i] == LINE) maxStart = i;
		/* end */
		else if(maxStart != -1 && (arr[i] == FLOOR || i == n - 1)){
			int len = i - maxStart;
			/* update maxLen */
			/* fine second longest paattern */
			if(maxLen < len && len <= max && endCheck != i){
				maxLen = len;
				maxEnd = i;
				
				/* return values */
				ret = maxLen;
				*end = maxEnd;
			}			
			/* ready to restart */
			maxStart = -1;
		}
		/* now is searching... */
		else if(maxStart == -1 && arr[i] == FLOOR) continue;
		/* now is measuring... */
		else if(maxStart != -1 && arr[i] == LINE) continue;
		
	}
	return ret;
}

int isConnected(vector<Point> vp, Mat m){
	if(vp.size() < 2) return -1;

	int ret = 1;
	int width = m.cols;
	int height = m.rows;
	uchar *data = (uchar *)m.data;

	Point start, end;
	start = vp.back(); vp.pop_back();
	end = vp.back(); vp.pop_back();
	int nDivide = 8;
	int count = 7;	// count-- if it is not line
	double dx = (end.x - start.x) / (double)nDivide;
	double dy = (end.y - start.y) / (double)nDivide;
	
	cout <<"Start x = "<<start.x<<endl;
	cout <<"Start y = "<<start.y<<endl;
	cout <<"End x = "<<end.x<<endl;
	cout <<"End y = "<<end.y<<endl;

	for(int i = 1; i < nDivide; i++){
		int idx = (int)(start.y + dy * i) * width + (int)(start.x + dx * i);
		
		if(data[idx] != LINE)
		{
			count--;
		}
		if(count==ERROR_COUNT){
			ret = 0;
			break;
		}
	}

	vp.push_back(end);
	vp.push_back(start);

	return ret;
}

vector<Point> getNodes(Mat grey){
			vector<Point> ret;
			int width = grey.cols;
			int height = grey.rows;
			Mat m = quantizeMat(grey);
			uchar *data = (uchar *)m.data;

			// create each edge's Pattern from frame
			uchar *topArray		= &data[0 * width 				+ 0];
			uchar *bottomArray	= &data[(height - 1) * width	+ 0];
			uchar *leftArray	= new uchar[height];
			uchar *rightArray	= new uchar[height];
			for(int i = 0; i < height; i++){
				leftArray[i] = data[i * width + 0]; 
				rightArray[i] = data[i * width + (width - 1)]; 
			}
			Pattern *top 	= new Pattern(topArray, width, Direction(TOP));
			Pattern *bottom = new Pattern(bottomArray, width, Direction(BOTTOM));
			Pattern *left 	= new Pattern(leftArray, height, Direction(LEFT));
			Pattern *right 	= new Pattern(rightArray, height, Direction(RIGHT));
			Pattern *pn[4] = { bottom, left, top, right};
			
			// find two patterns on each frame, not bottom(bottom is only one pattern) 
			int maxLen[8], maxEnd[8], mid[8];
			for(int i = 0; i < 8 ; i=i+2){
				if(maxLen[i] = findLongestPattern(pn[i/2], &maxEnd[i])){
					cout << "maxLen[i] = " << maxLen[i] << ", i = " << i <<endl;
					mid[i] = maxEnd[i] - maxLen[i] / 2;
					Point *pt = pn[i/2]->toPoint(mid[i]);
				
					// bottom only one pattern
					if(i != 0 && (maxLen[i+1] = findLongestPattern2(pn[i/2], &maxEnd[i+1], maxLen[i], maxEnd[i]))){
						cout << "maxLen[i+1] = " << maxLen[i+1] << ", i+1 = " << i+1 <<endl;
						mid[i+1] = maxEnd[i+1] - maxLen[i+1] / 2;
						Point *pt2 = pn[i/2]->toPoint(mid[i+1]);
						
						// push patterns clockwise
						if((i==0 && pt->x < pt2->x) || (i==2 && pt->y < pt2->y) || 
								(i==4 && pt2->x < pt->x) || (i==6 && pt2->y < pt->y)){
							ret.push_back(Point(pt2->x, pt2->y));
							cout<<"p["<<i<<"] = ("<<pt2->x<<", "<<pt2->y<<")"<<endl;	
							ret.push_back(Point(pt->x, pt->y));
							cout<<"p["<<i<<"] = ("<<pt->x<<", "<<pt->y<<")"<<endl;
						}
						else{
							ret.push_back(Point(pt->x, pt->y));
							cout<<"p["<<i<<"] = ("<<pt2->x<<", "<<pt2->y<<")"<<endl;	
							ret.push_back(Point(pt2->x, pt2->y));
							cout<<"p["<<i<<"] = ("<<pt->x<<", "<<pt->y<<")"<<endl;
						}
					}
					else{
						ret.push_back(Point(pt->x, pt->y));
						cout<<"p["<<i<<"] = ("<<pt->x<<", "<<pt->y<<")"<<endl;
					}
				}
			}


			// make one pattern if they are same pattern
			int vsize = ret.size();
			Point nodes[vsize+1];
			for(int i = vsize-1; i >= 0; i--){
				nodes[i] = ret.back();
				ret.pop_back();
			}
			nodes[vsize] = nodes[0];

			for(int i=0; i < vsize; i++){
				int dx = nodes[i].x - nodes[i+1].x;
				int dy = nodes[i].y - nodes[i+1].y;
				double distance = sqrt(dx*dx+dy*dy);
				if(distance<10){	// adjust 
					ret.push_back(Point((nodes[i].x+nodes[i+1].x)/2, (nodes[i].y+nodes[i+1].y)/2));
					cout<<"Make one pattern"<<endl;
					cout<<"i = "<<i<<", push_back("<<(nodes[i].x+nodes[i+1].x)/2<<", "<<(nodes[i].y+nodes[i+1].y)/2<<")"<<endl;
					i++;
				}
				else{
					cout<<"i = "<<i<<", push_back("<<nodes[i].x<<", "<<nodes[i].y<<")"<<endl;
					ret.push_back(Point(nodes[i].x, nodes[i].y));
				}
			}

			/* release */
			delete leftArray;
			delete rightArray;
			delete top;
			delete bottom;
			delete left;
			delete right;
			m.release();
	
			printf("ret size = %d\n", ret.size());
			return ret;
}

vector<Point> detectLine(Mat grey){
	vector<Point> ret;
	int width = grey.cols;
	int height = grey.rows;
	Mat m = quantizeMat(grey);
	uchar *data = (uchar *)m.data;
	
	/* create each edge's Pattern from frame */
	uchar *topArray		= &data[0 * width 				+ 0];
	uchar *bottomArray	= &data[(height - 1) * width	+ 0];
	uchar *leftArray	= new uchar[height];
	uchar *rightArray	= new uchar[height];
	uchar *halfArray	= &data[(height - 12) * width	+ 0];
	uchar *quaterArray	= &data[(height - 6) * width	+ 0];
	
	for(int i = 0; i < height; i++){
		leftArray[i] = data[i * width + 0]; 
		rightArray[i] = data[i * width + (width - 1)]; 
	}
	Pattern *top 	= new Pattern(topArray, width, Direction(TOP));
	Pattern *bottom = new Pattern(bottomArray, width, Direction(BOTTOM));
	Pattern *left 	= new Pattern(leftArray, height, Direction(LEFT));
	Pattern *right 	= new Pattern(rightArray, height, Direction(RIGHT));
	Pattern *half = new Pattern(halfArray, width, Direction(HALF));
	Pattern *quater = new Pattern(quaterArray, width, Direction(QUATER));
	Pattern *pn[6] = { bottom, left, top, right, half, quater};

	for(int i = 0; i < 4 && ret.size() < 2; i++){
		int maxLen[2], maxEnd[2], mid[2];
		int j;
		/* find logest and continuous pattern from each edges of frame */
		maxLen[0] = findLongestPattern(pn[i], &maxEnd[0]);
		cout<<"maxLen[0] = "<< maxLen[0] << ", i = " << i <<endl;
		if(maxLen[0] == 0) continue;
		
		/* push back middle point of the logest and continuous pattern */
		mid[0] = maxEnd[0] - maxLen[0] / 2;
		for(j = i + 1; j < 4 && ret.size() < 2; j++){
			maxLen[1] = findLongestPattern(pn[j], &maxEnd[1]); 
			cout<< "maxLen[1] = "<< maxLen[1] << ", j = " << j <<endl;
			if(maxLen[1] == 0 && i == 0 && j == 3){
				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< 3.1 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
				cout << "   Only bottom is connected. Anyway, it is connected too " <<endl; 
				
				// find half pattern
				maxLen[1] = findLongestPattern(pn[4], &maxEnd[1]); 
				if(maxLen[1] == 0){
					// find quater pattern
					maxLen[1] = findLongestPattern(pn[5], &maxEnd[1]); 
					if(maxLen[1] == 0){
						Point *pt = pn[0]->toPoint(mid[0]);
						if(pt == NULL) break;
						ret.push_back(Point(pt->x,1));
						ret.push_back(Point(pt->x,0));
						delete pt;	
						break;
					}
					else{
						mid[1] = maxEnd[1] - maxLen[1] / 2;
						Point *pt[2] = { pn[0]->toPoint(mid[0]), pn[5]->toPoint(mid[1]) };
						if(pt[0] == NULL || pt[1] == NULL) break;
						ret.push_back(Point(pt[0]->x, pt[0]->y));
						ret.push_back(Point(pt[1]->x, pt[1]->y));
						delete pt[0];
						delete pt[1];		
						break;
					}
				}
				else{
					mid[1] = maxEnd[1] - maxLen[1] / 2;
					Point *pt[2] = { pn[0]->toPoint(mid[0]), pn[4]->toPoint(mid[1]) };
					if(pt[0] == NULL || pt[1] == NULL) break;
					ret.push_back(Point(pt[0]->x, pt[0]->y));
					ret.push_back(Point(pt[1]->x, pt[1]->y));
					delete pt[0];
					delete pt[1];
					break;
				}
			}
			else if(maxLen[1] == 0 && i == 1 && j == 3){
				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< 3.2 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
				cout << "   Only left is connected. Anyway, it is connected too " <<endl; 
				
				ret.push_back(Point(15,1));
				ret.push_back(Point(15,0));
				break;
			}
			else if(maxLen[1] == 0 && i == 2 && j == 3){
				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< 3.3 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
				cout << "   Only top is connected. Anyway, it is connected too " <<endl; 
				
				// find half pattern
				maxLen[1] = findLongestPattern(pn[4], &maxEnd[1]); 
				if(maxLen[1] == 0){
					// find quater pattern
					maxLen[1] = findLongestPattern(pn[5], &maxEnd[1]); 
					if(maxLen[1] == 0){
						Point *pt = pn[2]->toPoint(mid[0]);
						if(pt == NULL) break;
						ret.push_back(Point(pt->x,1));
						ret.push_back(Point(pt->x,0));
						delete pt;	
						break;
					}
					else{
						mid[1] = maxEnd[1] - maxLen[1] / 2;
						Point *pt[2] = { pn[2]->toPoint(mid[0]), pn[5]->toPoint(mid[1]) };
						if(pt[0] == NULL || pt[1] == NULL) break;
						ret.push_back(Point(pt[1]->x, pt[1]->y));
						ret.push_back(Point(pt[0]->x, pt[0]->y));
						delete pt[0];
						delete pt[1];		
						break;
					}
				}
				else{
					mid[1] = maxEnd[1] - maxLen[1] / 2;
					Point *pt[2] = { pn[2]->toPoint(mid[0]), pn[4]->toPoint(mid[1]) };
					if(pt[0] == NULL || pt[1] == NULL) break;
					ret.push_back(Point(pt[1]->x, pt[1]->y));
					ret.push_back(Point(pt[0]->x, pt[0]->y));
					delete pt[0];
					delete pt[1];
					break;
				}
			}
			else if(maxLen[1] == 0)	continue;
				
			mid[1] = maxEnd[1] - maxLen[1] / 2;
			Point *pt[2] = { pn[i]->toPoint(mid[0]), pn[j]->toPoint(mid[1]) };
			if(pt[0] == NULL || pt[1] == NULL) break;
			ret.push_back(Point(pt[0]->x, pt[0]->y));
			ret.push_back(Point(pt[1]->x, pt[1]->y));
			
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< 3.4 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "                Check if line is connected " <<endl; 
			if(!isConnected(ret, m)){
				if(i == 0){
					cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< 3.4.1 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
					cout << "          Just go straigt. Anyway, it is connected too " <<endl; 
					
					ret.pop_back();
					ret.pop_back();
					Point *pt = pn[0]->toPoint(mid[0]);
					ret.push_back(Point(pt->x,1));
					ret.push_back(Point(pt->x,0));
					delete pt;
					break;
				}				
				else{
					cout << "<<<<<<<<<<<<<<<<<<<<<<<<<< 3.4.2 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
					cout << "                  Line is not connected " <<endl; 
					printf("%d, %d\n", pt[0]->x, pt[0]->y);
					printf("%d, %d\n", pt[1]->x, pt[1]->y);
					delete pt[1];  ret.pop_back();
					delete pt[0];  ret.pop_back();
				}
			}
			else{
				int dx = pt[0]->x - pt[1]->x;
				int dy = pt[0]->y - pt[1]->y;
				double distance = sqrt(dx*dx+dy*dy);
				if(distance<8){
					cout << "<<<<<<<<<<<<<<<<<<<<<<<<<< 3.4.3 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
					cout << "        Two patterns are too close. They are not connected " <<endl; 
					printf("%d, %d\n", pt[0]->x, pt[0]->y);
					printf("%d, %d\n", pt[1]->x, pt[1]->y);
					delete pt[1];  ret.pop_back();
					delete pt[0];  ret.pop_back();	
				}
				else{
					cout << "<<<<<<<<<<<<<<<<<<<<<<<<<< 3.4.4 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
					cout << "                     Line is conncted " <<endl; 
					printf("%d, %d\n", pt[0]->x, pt[0]->y);
					printf("%d, %d\n", pt[1]->x, pt[1]->y);
					delete pt[0];
					delete pt[1];	
					break;
				}
			}
		}
	}
	
	/* print */
	printData(m);

	/* release */
	delete leftArray;
	delete rightArray;
	delete top;
	delete bottom;
	delete left;
	delete right;
	delete half;
	delete quater;
	m.release();

	cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< 3.5 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
	cout << "                       Check vsize = " <<ret.size() <<endl; 
	return ret;
}

double calcAngle(vector<Point> vp, int num){
	int vpSize = vp.size(); // *CAUTION* vp.size() returns a unsigned value
	double ret = 0.0;
	double slope = 0.0;

	Point d;
	if(num == 2){
		d = vp[0] - vp[3];
		slope = (double)(vp[3].y-vp[0].y)/(double)(vp[3].x-vp[0].x);
		printf("vp[3] = (%d,%d), vp[0] = (%d,%d)\n", vp[3].x, vp[3].y, vp[0].x, vp[0].y);
	}
	else if(num == 1){
		d = vp[0] - vp[2];
		slope = (double)(vp[2].y-vp[0].y)/(double)(vp[2].x-vp[0].x);
		printf("vp[2] = (%d,%d), vp[0] = (%d,%d)\n", vp[2].x, vp[2].y, vp[0].x, vp[0].y);
	}
	else{
		d = vp[0] - vp[1];
		slope = (double)(vp[1].y-vp[0].y)/(double)(vp[1].x-vp[0].x);
		printf("vp[1] = (%d,%d), vp[0] = (%d,%d)\n", vp[1].x, vp[1].y, vp[0].x, vp[0].y);
	}
	cout << "dx: " << d.x << ", dy: " << d.y << endl;

	ret = acos(d.y / sqrt(d.x * d.x + d.y * d.y)) * 180.0 / PI;
//	cout << "slope: " << slope << ", d ret: " << ret << endl;

	if((slope<0)&&(ret<0))
		ret = -ret;
	else if((slope>0)&&(ret>0))
		ret = -ret;
//	cout << "slope: " << slope << ", d ret: " << ret << endl;

	if((d.y<=0)&&(d.x>0)&&(ret>0))
		ret = -ret;
	else if((d.y<=0)&&(d.x<0)&&(ret<0))
		ret = -ret;
//	cout << "slope: " << slope << ", d ret: " << ret << endl;
	return ret;
}
void dijkstra(int n, int start, int &end, int routeArray[], int &routeCount, int (*number)[NODE_NUM+1], bool nodeArray[]) {
	int i, vnear;
	int touch[n]; int length[n];
	int loop_count = n;
	int length_copy[n];
	routeCount = 0;

	for(i = 0; i < n; i++)
		length_copy[i] = INFINITE;
	for (i = 0; i < n; i++) { // For all vertices, initialize v1 to be the last
		touch[i] = start; // vertex on the current shortest path from v1,
		length[i] = number[start][i]; // and initialize length of that path to be the
//		cout <<"start = " << start << ", ";
//		cout <<"i = " << i << ", length[i] = " << length[i] <<endl;
	} // weight on the edge from v1.

	while(loop_count) { // Add all n-1 vertices to Y.
		int min = INFINITE;
		cout<<"loop_count = "<<loop_count<<endl;
		for (i = 0; i < n; i++){		    // Check each vertex for having shortest path.
			if (i == start) continue;
			if (0 < length[i] && length[i] < min) {	// among vertices not in Y
				min = length[i];
				vnear = i;
//				cout<<"min = "<<min<<", vnear = "<<vnear<<endl;
			}
		}
		
		if(min == INFINITE)	{
			loop_count--;
			continue;
		}
		else
			length_copy[vnear] = length[vnear];

		for (i = 0; i < n; i++){
			if (i == start) continue;
			if (length[vnear] + number[vnear][i] < length[i]) {			 	// length[i] in Y <0
				length[i] = length[vnear] + number[vnear][i]; 
				touch[i] = vnear;											// For each vertex not in Y, update its shortest
//				number[start][i] = length[vnear] + number[vnear][i];
//				number[i][start] = length[vnear] + number[vnear][i];
			}																// path. Add vertex indexed by vnear to Y
		}
		length[vnear] = -1;
		loop_count--;
	}

	// finde shortest node
	int min = INFINITE;
	for(i = 0; i < n; i++){
//		cout<<"length_copy[i] = " <<length_copy[i] <<endl;
		if(min > length_copy[i] && nodeArray[i] == false){
			end = i;
			min = length_copy[i];
		}
	}
	cout <<"end = " <<end<<endl;	
	cout <<"min = " <<min<<endl;
	routeArray[routeCount] = end;
	routeCount++;
	while (1){
		if (touch[end] == start){
			break;
		}
		else{
			routeArray[routeCount] = touch[end];
			end = touch[end];
			routeCount++;
		}
	}
	for (i = 0; i < routeCount; i++)
		cout << "routeArray[" << i << "] = " << routeArray[i] << endl;
}


//MAIN//////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[]) {
	/* Initialize variables for speed*/
	int fd;
	double speed_left = 0;
	double speed_right = 0;
	double speed_delta = 0;
	double speed_adj = 0;
	double backing_speed_left = 0;
	double backing_speed_right = 0;
	double qr_speed_left = 0;
	double qr_speed_right = 0;

	/* Initialize variables for time*/
	int backing_time = 0;
	int qr_time = 0;
	int start_time = 10;
	
	/* Initialize variables for Modes */
	bool readyMode = true;
	bool gobackMode = false;
	bool goMode = false;
	bool endMode = false;

	/* Initialize variables for grape*/
	bool nodeArray[NODE_NUM];

	int nodeTrace[MAXIMUM];
	int nodeCount = 0;
	int traceCount = 0;
	int nodeMatrix[NODE_NUM+1][NODE_NUM+1];
	
	for(int i=0; i<NODE_NUM; i++)
		for(int j=0; j<NODE_NUM; j++)
			nodeMatrix[i][j] = INFINITE;
	for(int i=0;i<MAXIMUM;i++)
		nodeTrace[i] = 0;
	for(int i=0;i<NODE_NUM;i++)
		nodeArray[i] = false;
	
	/* Initialize variables for dijkstra algorithm*/
	int routeArray[MAXIMUM];
	int routeCount = 0;

	/* Initialize variables for QRcode*/
	Point qr_centralPoint;
	string qr_dataString;
	int qr_dataIntArray[MAXIMUM];
	
	/* open the EV3 */
	if((fd = open(EV3_DEV_NAME, O_RDWR | O_SYNC)) < 0) {
		printf("open error!\n");
		return -1;
	}
	
	/* init EV3 motors */
	speed_left = speed_right = 0;
	update_motor_speed(fd, speed_left, speed_right);

	/* open a video dev	 */
	VideoCapture cap(CAM_NUM);
	if (!cap.isOpened()){
		cout << "Cannot open the video cam" << endl;
		return -1;
	}

	/* set a capturing resolution */
	cap.set(CV_CAP_PROP_CONTRAST, CAP_CONSTRAST);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, CAP_WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT);
	cap.set(CV_CAP_PROP_FPS,CAP_FPS );

	/* make a QR code scanner */
	ImageScanner scanner;
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

	/* confirm a capturing resolution */
	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<< 0 >>>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
	cout << "					   Setting is done	"<<endl;
	cout << "Frame Size: " << dWidth << " * " << dHeight << endl;

	/* 
	 * create a window called "MyVideo" 
	 * this fuction requires GUI
	 */
#ifdef GUI
	namedWindow("INPUT", CV_WINDOW_AUTOSIZE);
	namedWindow("OUTPUT", CV_WINDOW_AUTOSIZE);
#endif
	
	/* Variables for checking if EV3 detected QR code well */
	int temp_angle[100];
	int temp_node[100];
	int temp_count = 0;
	while (1){
		cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<< 1 >>>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
		cout << "                         Loop start 	"<<endl;
		
		/* read a new frame from video */
		Mat frame;
		if (!cap.read(frame)){
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		/* resize and wipe out colors from the frame */
		Mat small(SMALL_WIDTH, SMALL_HEIGHT, CV_8UC3);
		Mat grey(SMALL_WIDTH, SMALL_HEIGHT, CV_8UC1);
		Mat grey2(SMALL_WIDTH, SMALL_HEIGHT, CV_8UC1);
		resize(frame, small, Size(SMALL_WIDTH, SMALL_HEIGHT), 0, 0, 0);
		cvtColor(small, grey, CV_BGR2GRAY);
		cvtColor(small, grey2, CV_BGR2GRAY);
		// small Mat will be unnecessary any more

/////////////////////////////////////////////////////////////////////////////////////////////
		/* QRCODE */
		/* wipe out colors from the frame */
		Mat grey_qr;
		cvtColor(frame, grey_qr, CV_BGR2GRAY);

		/* wrap image data */
		int width = frame.cols;
		int height = frame.rows;
		uchar *raw = (uchar *)grey_qr.data;
		Image image(width, height, "Y800", raw, width * height);

		/* scan the image for barcodes */
		int n = scanner.scan(image); // zbar
		//cout << "scanner.scan(image) == " << n << endl;
		qr_dataString.clear();
		cout << qr_dataString <<endl;
			
		/* extract results */
		for(Image::SymbolIterator symbol = image.symbol_begin();  // zbar
			symbol != image.symbol_end();  // zbar
			++symbol)
		{
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<< 2 >>>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "                   QR code is dectected	"<<endl;
			
			/* do something useful with results */
			static int count = 0;
			count++;
			cout << "Count:\t" << count << ", ";
	
			/* get symbol's type and data */
			cout << "Decoded " << symbol->get_type_name() <<  // zbar
				" symbol: " << symbol->get_data() << endl; // zbar
			qr_dataString = symbol->get_data();
		
			/* change qr code */
			if(CAL_MODE){
				if(!qr_dataString.compare("0"))
					qr_dataString = "0 2 1 4 5 12 5 1 5 1 5 1";
				if(!qr_dataString.compare("1"))
					qr_dataString = "1 3 0 4 2 8 3 9 0 2 3 0 2 3 0 2 3 0 2 3";
				if(!qr_dataString.compare("2"))
					qr_dataString = "2 3 1 8 5 5 4 4 1 5 4 1 5 4 1 5 4 1 5 4";
				if(!qr_dataString.compare("3"))
					qr_dataString = "3 2 1 9 4 10 1 4 1 4 1 4";
				if(!qr_dataString.compare("4"))
					qr_dataString = "4 3 2 4 3 10 5 3 3 2 5 3 2 5 3 2 5 3 2 5";
				if(!qr_dataString.compare("5"))
					qr_dataString = "5 3 0 12 4 3 2 5 0 4 2 0 2 0 4 2 4 2 0 4";
			}
			else{
				if(!qr_dataString.compare("0"))
					qr_dataString = "0 2 1 4 5 12 5 1 -115 5 -180 1 5 115 1 -180";
				if(!qr_dataString.compare("1"))
					qr_dataString = "1 3 0 4 2 8 3 9 0 2 -90 3 40 0 -180 2 3 -50 0 90 2 -180 3 0 -40 2 50 3 -180";
				if(!qr_dataString.compare("2"))
					qr_dataString = "2 3 1 8 5 5 4 4 1 5 -40 4 50 1 -180 5 4 -90 1 40 5 -180 4 1 -55 5 90 4 -180";
				if(!qr_dataString.compare("3"))
					qr_dataString = "3 2 1 9 4 10 1 4 -30 1 -180 4 1 30 4 -180";
				if(!qr_dataString.compare("4"))
					qr_dataString = "4 3 2 4 3 10 5 3 2 5 -115 3 45 2 -180 3 2 -45 5 20 3 -180 5 3 -20 2 120 5 -180";
				if(!qr_dataString.compare("5"))
					qr_dataString = "5 3 0 12 4 3 2 4 2 0 -130 4 90 2 -180 0 4 40 2 130 0 -180 4 0 -35 2 -90 4 -180";
			}

			/* get symbol's position and angle */
			vector<Point> vp;
			int m = symbol->get_location_size(); // zbar
			cout << "symbol->get_location_size() == " << m << endl;
			for(int i = 0; i < m; i++){
				cout<<"x = "<<symbol->get_location_x(i)<<", y = "<<symbol->get_location_y(i)<<endl;
				vp.push_back(
					Point(symbol->get_location_x(i), symbol->get_location_y(i))); // zbar
			}
			qr_centralPoint.x = (vp[0].x + vp[2].x)/2;
			qr_centralPoint.y = (vp[0].y + vp[2].y)/2;
			// 640*480 -> 32*24 
			qr_centralPoint.x = qr_centralPoint.x/20;
			qr_centralPoint.y = qr_centralPoint.y/20;
			cout<<"QR central point = "<<qr_centralPoint.x<<", "<<qr_centralPoint.y<<endl;
			
			RotatedRect r = minAreaRect(vp);
			Point2f pts[4];
			r.points(pts);
			for(int i = 0; i < 4; i++){
				line(frame, pts[i], pts[(i + 1) % 4], Scalar(255,0,0), 3);
			}
			cout << "QRcode Angle: " << r.angle << endl;
		}
/////////////////////////////////////////////////////////////////////////////////////////////
		
		/* detect lines */
		cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<< 3 >>>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
		cout << "                        Detect Line	"<<endl;
		vector<Point> vp = detectLine(grey);
		int vpSize = (int)vp.size();

#ifdef GUI
		/* draw lines */
		cout << "vp.size(): " << vpSize << endl;
		for(int i = 0; i < vpSize - 1; i+=2){
			line(small, vp[i], vp[i + 1], Scalar(0, 0, 255), 2);
		}
		
		/* 
		 * show the frame in "MyVideo" window 
		 * this fuction requires GUI
		 */
		Mat output(CAP_WIDTH, CAP_HEIGHT, CV_8UC3);
		resize(small, output, Size(CAP_WIDTH, CAP_HEIGHT), 0, 0, 0);
		imshow("INPUT", frame); 
		imshow("OUTPUT", output); 
		output.release();
#endif

		/* release */
		frame.release();
		small.release();
		grey_qr.release();
		grey.release();
		
		/* starting condition & remove noise during start_time when start*/
		if(speed_left == 0 && speed_right == 0 && vpSize == 0 || start_time != 0){
			update_motor_speed(fd, SPEED_MOTOR_SAT/2, SPEED_MOTOR_SAT/2);
			start_time--;
			continue;
		}
		
		/* control the EV3 */
		cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<< 4 >>>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
		cout << "1) Backing   2) QRmoving   3) QRcode is detected   4)Line is detected   5) Line is not dectected and finished"<<endl;
		if(backing_time){
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< 4.1 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "                          backing " <<endl; 
			cout << "backing_time = "<<backing_time<<endl;
			update_motor_speed(fd, backing_speed_left, backing_speed_right);
			backing_time--;
			
			if(backing_time == 4)
			{
				backing_speed_left = -60;
				backing_speed_right = -60;
			}
			if(backing_time == 2)
			{
				backing_speed_left = 57.5;
				backing_speed_right = 25;
			}	
		}
		else if(qr_time){
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< 4.2 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "                         QR moving " <<endl; 
			cout << "qr_time = "<<qr_time<<endl;
			update_motor_speed(fd, qr_speed_left, qr_speed_right);
			qr_time--;
		}
		else if(qr_dataString.compare("")){
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< 4.3 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "                    QRcode is dectected " <<endl; 		
	
			int i = 1; int j = 0; int edgeNum;
			int qr_dataIntArray[MAXIMUM];
			char *qr_dataPointer = new char[qr_dataString.length()+1];
			char *temp;

			/* change qr stirng to qr integer */
			strcpy(qr_dataPointer, qr_dataString.c_str());
			temp = strtok(qr_dataPointer, " ");
			if(temp){	
				qr_dataIntArray[0] = atoi(temp);
				cout<<"qr_dataIntArray[0] = "<<qr_dataIntArray[0]<<endl;
			}
			while(temp!=NULL){
				temp = strtok(NULL, " ");
				if(temp){
					qr_dataIntArray[i] = atoi(temp);
					cout<<"qr_dataIntArray["<<i<<"] = "<<qr_dataIntArray[i]<<endl;
					i++;
				}
			}
			temp = NULL; 
			edgeNum = qr_dataIntArray[1];

			/* Add grape information to matrix */
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<< 4.3.1 >>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "                         nodeMatrix " <<endl; 
			for(int i=0; i<edgeNum; i++){
				nodeMatrix[qr_dataIntArray[0]][qr_dataIntArray[2+i*2]] = qr_dataIntArray[3+i*2];				
				nodeMatrix[qr_dataIntArray[2+i*2]][qr_dataIntArray[0]] = qr_dataIntArray[3+i*2];				
			}
			for(int i=0; i<NODE_NUM; i++){
				for(int j=0; j<NODE_NUM; j++)
					printf("%5d ", nodeMatrix[i][j]);
				cout<<endl;
			}

			/* Trace node */
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<< 4.3.2 >>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "                         nodeTrace " <<endl; 
			if(traceCount == 0){
				nodeTrace[traceCount] = qr_dataIntArray[0];
				traceCount++;
			}
			else{
				// make QRcode detected only one time 
				if(qr_dataIntArray[0] != nodeTrace[traceCount-1]){
					nodeTrace[traceCount] = qr_dataIntArray[0];
					traceCount++;
				}
			}
			for(int i=0; i<traceCount; i++)
				cout << nodeTrace[i] <<" -> ";
			cout << "END" <<endl;

			/* Count node*/
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<< 4.3.3 >>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "                         nodeArray " <<endl; 
			// check if the node is already passed
			if(nodeArray[qr_dataIntArray[0]] == false){
				nodeArray[qr_dataIntArray[0]] = true;
				nodeCount++;
			}
			for(int i=0; i<NODE_NUM; i++){
				if(nodeArray[i] == true)
					cout<<i<<", ";
			}
			cout << "END" <<endl;				
			cout << "nodeCount = " << nodeCount <<endl;	
			
			/* change mode among readyMode, gobackMode, and goMode */ 	
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<< 4.3.4 >>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "     change mode among redayMode, gobackMode, and goMode " <<endl; 
			if(readyMode){
				// when all nodes are passed 
				if( nodeCount == NODE_NUM ){
					readyMode = false;
					gobackMode = true;
					int start = nodeTrace[traceCount-1];
					int end = START_NODE;
					cout << "Go from startnode to endnode as shortest path" <<endl;
					cout << "start = "<<start<<endl;
					cout << "end = "<<end<<endl;
					dijkstra(NODE_NUM, start, end, routeArray, routeCount, nodeMatrix, nodeArray);
					cout << "gobackMode routeCount = " << routeCount <<endl;	
				}
			}	
			else if(gobackMode){
				// when all routes are paseed
				if(routeCount == 0){
					gobackMode = false;
					goMode = true;
					int start = START_NODE;
					int end = END_NODE;
					cout << "Go from startnode to endnode as shortest path" <<endl;
					cout << "start = "<<start<<endl;
					cout << "end = "<<end<<endl;
					dijkstra(NODE_NUM, start, end, routeArray, routeCount, nodeMatrix, nodeArray);
					cout << "goMode routeCount = " << routeCount <<endl;	
				}	
			}
			else if(goMode){
				// when all routes are passed
				if(routeCount == 0 || qr_dataIntArray[0] == END_NODE){
					cout << "Finally, we arrived in endnode! " <<endl;
					cout << "Everything is done. Bye " <<endl;
					for(int i=0; i<NODE_NUM; i++){
						for(int j=0; j<NODE_NUM; j++)
							printf("%5d ", nodeMatrix[i][j]);
						cout<<endl;
					}
					cout<<"ALL NODES AND ANGLES passed"<<endl;
					for(int i=0; i<temp_count; i++)
						cout<<"node = "<<temp_node[i]<<", angle = "<<temp_angle[i]<<endl;
					break;
				}
			}
			
			/* Calculate degrees */
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<< 4.3.5 >>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "                     calculate degrees " <<endl; 
			int startIndex = 2 + edgeNum*2;	
			int beforeNode;
			double angle = INFINITE;
			int eachGap;
			if(CAL_MODE)		
				eachGap = 1 + edgeNum;
			else
				eachGap = 1 + edgeNum*2;
			vector<Point> vp = getNodes(grey2);
			
			// calculate degrees from qr_centralPoint
			vp[0].x = qr_centralPoint.x;
			vp[0].y = qr_centralPoint.y;

			/* Select next node */
			if(readyMode){
				/* Select shorest node(next node) */
				cout << "                      select shorest node" <<endl; 
				
				int start = nodeTrace[traceCount-1];
				int end;
				dijkstra(NODE_NUM, start, end, routeArray, routeCount, nodeMatrix, nodeArray);
				int minNode = routeArray[routeCount-1];
				cout<<"minNode(end) = "<<end<<endl;
				cout<<"next node = routeArray[routeCount-1] = "<<routeArray[routeCount-1]<<endl;

				// when we met qrcode for the first time
				if(nodeCount == 1){
					angle = START_ANGLE;		
					cout << "start angle: " << angle << endl;
					temp_angle[temp_count] = angle;
					temp_node[temp_count] = 1;	
					temp_count++;
				}
				else{
					beforeNode = nodeTrace[traceCount-2];
					cout<<"beforeNode = "<<beforeNode<<endl;
					
					if(CAL_MODE){	
						for(int i=0; i<edgeNum; i++){
							if(beforeNode == qr_dataIntArray[startIndex + i*eachGap]){
								for(int j=0; j<edgeNum; j++){
									if(minNode == qr_dataIntArray[startIndex + i*eachGap + 1 + j]){
										// when return
										if(minNode == beforeNode)
											angle = -180;
										else
											angle = calcAngle(vp, j);
										cout << "angle: " << angle << endl;
										
										temp_angle[temp_count] = angle;
										temp_node[temp_count] = minNode;
										temp_count++;
										break;
									}
								}
							}
						}
					}
					else{
						for(int i=0; i<edgeNum; i++){
							if(beforeNode == qr_dataIntArray[startIndex + i*eachGap]){
								for(int j=0; j<edgeNum; j++){
									if(minNode == qr_dataIntArray[startIndex + i*eachGap + 1 + j*2]){
										angle = qr_dataIntArray[startIndex + i*eachGap + 2 + j*2];
										cout << "angle: " << angle << endl;
									
										temp_angle[temp_count] = angle;
										temp_node[temp_count] = minNode;
										temp_count++;
										break;
									}
								}
							}
						}
					}
					if(angle == INFINITE){
						angle = calcAngle(vp, 0);
						cout << "Failed to dectect qrcode before " << endl;
						cout << "angle =  " << angle << endl;
								
						temp_angle[temp_count] = angle;
						temp_count++;
					}
				}
			}
			else if(gobackMode){
				/* Select fixed node(next node) */
				cout << "                     go back to startnode " <<endl; 
				cout << "                       select fixed node " <<endl; 	
				
				beforeNode = nodeTrace[traceCount-2];
				cout<<"beforeNode = "<<beforeNode<<endl;
				
				if(CAL_MODE){	
					for(int i=0; i<edgeNum; i++){
						if(beforeNode == qr_dataIntArray[startIndex + i*eachGap]){
							for(int j=0; j<edgeNum; j++){
								if(routeArray[routeCount-1] == qr_dataIntArray[startIndex + i*eachGap + 1 + j]){
									// when return
									if(routeArray[routeCount-1] == beforeNode)
										angle = -180;
									else
										angle = calcAngle(vp, j);
									cout << "angle = " << angle << endl;	
									cout << "node = " << routeArray[routeCount-1] <<endl;
									cout << "routeCount = " << routeCount <<endl;	
									
									temp_angle[temp_count] = angle;
									temp_node[temp_count] = routeArray[routeCount-1];
									temp_count++;
								
									// make detect qrcode only one time
									if(qr_dataIntArray[0] != beforeNode)
										routeCount--;
									break;
									}
							}
						}
					}
				}	
				else{
					for(int i=0; i<edgeNum; i++){ 
						if(beforeNode == qr_dataIntArray[startIndex + i*eachGap]){
							for(int j=0; j<edgeNum; j++){
								if(routeArray[routeCount-1] == qr_dataIntArray[startIndex + i*eachGap + 1 + j*2]){
									angle = qr_dataIntArray[startIndex + i*eachGap + 2 + j*2];
									cout << "angle: " << angle << endl;
										
									temp_angle[temp_count] = angle;
									temp_node[temp_count] = routeArray[routeCount-1];
									temp_count++;
									if( qr_dataIntArray[0] != beforeNode)
										routeCount--;
									break;
								}
							}
						}
					}	
				}
				
				if(angle == INFINITE){
					angle = calcAngle(vp, 0);
					cout << "Failed to dectect qrcode before " << endl;
					cout << "angle =  " << angle << endl;
					cout << "routeCount = " << routeCount <<endl;	
								
					temp_angle[temp_count] = angle;
					temp_count++;
				}		
			}
			else if(goMode){
				/* Select fixed node(next node) */
				cout << "                  go from startnode to endnode " <<endl; 
				cout << "                       select fixed node " <<endl; 
				beforeNode = nodeTrace[traceCount-2];
				cout<<"beforeNode = "<<beforeNode<<endl;
				if(CAL_MODE){
					for(int i=0; i<edgeNum; i++){
						if(beforeNode == qr_dataIntArray[startIndex + i*eachGap]){
							for(int j=0; j<edgeNum; j++){
								if(routeArray[routeCount-1] == qr_dataIntArray[startIndex + i*eachGap + 1 + j]){
									// when return
									if(routeArray[routeCount-1] == beforeNode)
										angle = -180;
									else
										angle = calcAngle(vp, j);
									cout << "angle = " << angle << endl;	
									cout << "node = " << routeArray[routeCount-1] <<endl;
									cout << "routeCount = " << routeCount <<endl;	
									
									temp_angle[temp_count] = angle;
									temp_node[temp_count] = routeArray[routeCount-1];
									temp_count++;
									if(qr_dataIntArray[0] != beforeNode)
										routeCount--;
									break;
								}
							}
						}
					}
				}
				else{
					for(int i=0; i<edgeNum; i++){
						if(beforeNode == qr_dataIntArray[startIndex + i*eachGap]){
							for(int j=0; j<edgeNum; j++){
								if(routeArray[routeCount-1] == qr_dataIntArray[startIndex + i*eachGap + 1 + j*2]){
									angle = qr_dataIntArray[startIndex + i*eachGap + 2 + j*2];
									cout << "angle: " << angle << endl;
								
									temp_angle[temp_count] = angle;
									temp_node[temp_count] = routeArray[routeCount-1];
									temp_count++;
									if( qr_dataIntArray[0] != beforeNode)
										routeCount--;
									break;
								}
							}
						}
					}	
				}
				if(angle == INFINITE){
					angle = calcAngle(vp, 0);
					cout << "QRcode is not detected." <<endl;
					cout << "angle =  " << angle << endl;
					cout << "routeCount = " << routeCount <<endl;	
								
					temp_angle[temp_count] = angle;
					temp_count++;
				}		
 			}

 			/* modify heading  */
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<< 4.3.6 >>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "                     modifing heading " <<endl; 
			
			/* Moving QRspeed 6 times more or Backingspeed 6 times more*/
			if(angle == -180){
				backing_time = 6;
				backing_speed_left = BACK_SPEED_LEFT;
				backing_speed_right = BACK_SPEED_RIGHT;
				cout<<"backing_speed_left = "<<backing_speed_left<<endl;
				cout<<"backing_speed_right = "<<backing_speed_right<<endl;
				update_motor_speed(fd, backing_speed_left, backing_speed_right);				
				continue;
			}
			else
				qr_time = 6;
		
			// adjust
			if(-45 < angle || angle < 45)	
				speed_adj = QR_SPEED_ADJ+0.1;
			else if(angle < -85 || 85 < angle)
				speed_adj = QR_SPEED_ADJ-0.1;
			else
				speed_adj = QR_SPEED_ADJ;

			speed_delta = angle/90*(double)SPEED_MOTOR_SAT;
			cout << "speed_adj: " << speed_adj << endl;
			cout << "speed_delta: " << speed_delta << endl;
			
			speed_left = SPEED_MOTOR_SAT + speed_adj*speed_delta;		
			speed_right = SPEED_MOTOR_SAT - speed_adj*speed_delta;
			if(speed_right<0)	
				speed_right = 0;
			if(speed_left<0)
				speed_left = 0;
			qr_speed_left = speed_left;
			qr_speed_right = speed_right;
			update_motor_speed(fd, speed_left, speed_right);
			
			qr_dataString.clear();
		}
		else if(vpSize!=0){
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< 4.4 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "                   Line is connected " <<endl; 
			/* running condition */
			/* calculate angles */
			double angle = calcAngle(vp, 0);
			
			if ((-15 < angle) && (angle < 15) && (vp[0].x <  14)){
				if(vp[0].x < 8)
					angle = -25;
				else
					angle = -15;
				cout<<"Go left more"<<endl;
			}
			else if ((-15 < angle) && (angle < 15) && (vp[0].x > 17)){
				if(vp[0].x > 23)
					angle = 25;
				else
					angle = 15;
				cout<<"Go right more"<<endl;
			}
			
			if(angle>0)	
				speed_adj = SPEED_ADJ*(1+angle/90);
			else 
				speed_adj = SPEED_ADJ*(1-angle/90);
			speed_delta = angle/90*(double)SPEED_MOTOR_SAT;

			cout << "angle: " << angle << endl;
			cout << "speed_adj: " << speed_adj << endl;
			cout << "speed_delta: " << speed_delta << endl;
			if(angle>0){
			 	speed_left = SPEED_MOTOR_SAT + speed_adj*speed_delta;		
				speed_right = SPEED_MOTOR_SAT - 1.2*speed_adj*speed_delta;
				if(speed_right<0)
					speed_right = 0;
				update_motor_speed(fd, speed_left, speed_right);
			}
			else{
			 	speed_left = SPEED_MOTOR_SAT + 1.2*speed_adj*speed_delta;		
				speed_right = SPEED_MOTOR_SAT - speed_adj*speed_delta;
				if(speed_left<0)
					speed_left = 0;
				update_motor_speed(fd, speed_left, speed_right);	
			}
		}
		else{
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< 4.5 >>>>>>>>>>>>>>>>>>>>>>>>>>>> " <<endl;
			cout << "           Line is not dectected and finished " <<endl; 
			
			for(int i=0; i<NODE_NUM; i++){
				for(int j=0; j<NODE_NUM; j++)
					printf("%5d ", nodeMatrix[i][j]);
				cout<<endl;
			}
			cout<<"ALL NODES AND ANGLES"<<endl;
			for(int i=0; i<temp_count; i++)
				cout<<"node = "<<temp_node[i]<<", angle = "<<temp_angle[i]<<endl;
	 		break;
		}
		
		/* 
		 * wait for 'esc' key press for 10ms. 
		 * If 'esc' key is pressed, break loop 
		 */
		if (waitKey((int)(1.0 / CAP_FPS * 1000.0) * 0.8) == WAIT_KEY){
			cout << "ESC key is pressed by user" << endl;
			break;
		}
	}
	
	/* finalize the EV3 */
	update_motor_speed(fd, 0, 0);
   	close(fd);
    return 0;
}




