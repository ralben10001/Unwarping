 
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <algorithm>
#include <vector>
 
#include <stdio.h>
#include "ARToolKitPlus/TrackerSingleMarkerImpl.h"
 


 
CvScalar target_color[4] = { // in BGR order
		{{   0,   0, 255,   0 }},  // red
		{{   0, 255,   0,   0 }},  // green
		{{ 255,   0,   0,   0 }},  // blue
		{{   0, 255, 255,   0 }}   // yellow
};
ARToolKitPlus::ARMarkerInfo * marker_info;
using namespace std;
using namespace cv;

bool isHEdge(IplImage* out, int i, int j, int amax, int dir) {
	bool retVal = true;
	if (amax == 0) {
		if (cvGet2D(out,j,i).val[0] == 255)
			return false;
		return true;
	}
	int count = 0;
	//bool out1 = true;
	//bool out2 = true;
	for (int a = 0; a < amax; a++) {
		if (cvGet2D(out,j+a,i+dir).val[0] == 255) {
			retVal &= isHEdge(out,i+dir,j+a,0,dir);
			count++;
		}
		if (cvGet2D(out,j-a,i+dir).val[0] == 255) {
			retVal &= isHEdge(out,i+dir,j-a,0,dir);
			count++;
		}
	}
	if (count > 1)
		return false;
	else
		return retVal;
}

bool isVEdge(IplImage* out, int i, int j, int amax, int dir) {
	bool retVal = true;
	if (amax == 0) {
		if (cvGet2D(out,j,i).val[0] == 255)
			return false;
		return true;
	}
	int count = 0;
	//bool out1 = true;
	//bool out2 = true;
	for (int a = 0; a < amax; a++) {
		if (cvGet2D(out,j+dir,i+a).val[0] == 255) {
			retVal &= isVEdge(out,i+a,j+dir,0,dir);
			count++;
		}
		if (cvGet2D(out,j+dir,i-a).val[0] == 255) {
			retVal &= isVEdge(out,i-a,j+dir,0,dir);
			count++;
		}
	}
	if (count > 1)
		return false;
	else
		return retVal;
}


class MyLogger : public ARToolKitPlus::Logger
{
    void artLog(const char* nStr)
    {
        printf(nStr);
    }
};
void ARstuff() {
   marker_info = new ARToolKitPlus::ARMarkerInfo[200];
	    const bool    useBCH = true;
		IplImage* adaptiveImg;
		
	int unwarped[8][8];

    const int     width = 640, height = 480, bpp = 1,
                  numPixels = width*height*bpp;
    size_t        numBytesRead;

    MyLogger      logger;

    ARToolKitPlus::TrackerSingleMarker *tracker = new ARToolKitPlus::TrackerSingleMarkerImpl<6,6,6,
                                                         ARToolKitPlus::PIXEL_FORMAT_LUM, 1, 180>(width,height);
	
    tracker->setLogger(&logger);

    tracker->init("no_distortion.cal", 1.0f, 1000.0f);   

 
	tracker->setBorderWidth(0.125f );
    tracker->setThreshold(150);

    tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

    tracker->setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);

	 CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
   if ( !capture ) {
     fprintf( stderr, "ERROR: capture is NULL \n" );
     getchar();
     return;
   }
   cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
   cvNamedWindow( "mywindowb", CV_WINDOW_AUTOSIZE );
   
   IplImage* grayImage;
   while ( 1 ) {
     IplImage* frame = cvQueryFrame( capture );
	 
     if ( !frame ) {
       fprintf( stderr, "ERROR: frame is null...\n" );
       getchar();
       break;
     }
	 grayImage = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );

     cvCvtColor( frame, grayImage, CV_BGR2GRAY );
	    adaptiveImg = cvCreateImage(cvSize(grayImage->width,grayImage->height),IPL_DEPTH_8U, 1);
   cvAdaptiveThreshold( grayImage, adaptiveImg, 255,
                         CV_ADAPTIVE_THRESH_MEAN_C , CV_THRESH_BINARY,
                          15, -4 );
   
   int marker_num;

   tracker->arDetectMarkerLite((unsigned char*)(adaptiveImg->imageData), 160, &marker_info, &marker_num);
   for (int i = 0; i < marker_num; i++) {
	   int thresh = 0;
	   int lowx = min(min(min(marker_info[i].vertex[0][0],marker_info[i].vertex[1][0]),marker_info[i].vertex[2][0]),marker_info[i].vertex[3][0]);
	   int lowy = min(min(min(marker_info[i].vertex[0][1],marker_info[i].vertex[1][1]),marker_info[i].vertex[2][1]),marker_info[i].vertex[3][1]);
	   int highx = max(max(max(marker_info[i].vertex[0][0],marker_info[i].vertex[1][0]),marker_info[i].vertex[2][0]),marker_info[i].vertex[3][0]);
	   int highy = max(max(max(marker_info[i].vertex[0][1],marker_info[i].vertex[1][1]),marker_info[i].vertex[2][1]),marker_info[i].vertex[3][1]);
	   int totalpixs = (highx-lowx)*(highy-lowy);
	   for (int a = lowx; a < highx; a++) {
		   for (int b = lowy; b < highy; b++) {
			   thresh += cvGet2D(grayImage,b,a).val[0];
		   }
	   }
	   thresh = (int)floor((float)thresh/totalpixs);
	   CvScalar black;
	   black.val[0] = 0;
	   CvScalar white;
	   white.val[0] = 255;
	   for (int a = lowx; a < highx; a++) {
		   for (int b = lowy; b < highy; b++) {
			   if (cvGet2D(grayImage,b,a).val[0] < thresh)
				   cvSet2D(adaptiveImg,b,a,black);
		   }
	   }
   }
   if (marker_num == 0)
	   continue;
    for( int i = 0; i < 4; i++) {
			int radius = 480/50;

			cvCircle(grayImage,
					cvPoint((int)(marker_info[0].vertex[i][0] + 0.5f),(int)(marker_info[0].vertex[i][1] + 0.5f)),
					radius,
					target_color[0]);
		}
	
	
     cvShowImage( "mywindow", grayImage);

     if ( (cvWaitKey(10) & 255) == 27 ) break;
   }
   float xlength = sqrt(pow(marker_info[0].vertex[1][0]-marker_info[0].vertex[0][0],2) + pow(marker_info[0].vertex[1][1]-marker_info[0].vertex[0][1],2));
	
	float ylength = sqrt(pow(marker_info[0].vertex[2][0]-marker_info[0].vertex[0][0],2) + pow(marker_info[0].vertex[2][1]-marker_info[0].vertex[0][1],2));

   for (int i = 0; i < 8;i++) {
		unwarped[i][0] = 0;
		unwarped[i][7] = 0;
		unwarped[0][i] = 0;
		unwarped[7][i] = 0;
	}

	ARFloat xaxis[2]; 
	xaxis[0]= marker_info[0].vertex[0][0] - marker_info[0].vertex[1][0];
	xaxis[1]= marker_info[0].vertex[0][1] - marker_info[0].vertex[1][1];
	ARFloat yaxis[2];
	yaxis[0] = marker_info[0].vertex[0][0] - marker_info[0].vertex[3][0];
	yaxis[1] = marker_info[0].vertex[0][1] - marker_info[0].vertex[3][1];
	xaxis[0] /= -8;
	xaxis[1] /= -8;
	yaxis[0] /= -8;
	yaxis[1] /= -8;
	int startx = floor(marker_info[0].vertex[0][0]+xaxis[0]*0.5 + yaxis[0]*0.5);
	
	int starty = floor(marker_info[0].vertex[0][1]+xaxis[1]*0.5 + yaxis[1]*0.5);
	float xint = floor(xlength/8);
	float yint = floor(ylength/8);
	int currx,curry;
	int rad = min(xint,yint)/2;

	int lowx = min(min(min(marker_info[0].vertex[0][0],marker_info[0].vertex[1][0]),marker_info[0].vertex[2][0]),marker_info[0].vertex[3][0]);
	   int lowy = min(min(min(marker_info[0].vertex[0][1],marker_info[0].vertex[1][1]),marker_info[0].vertex[2][1]),marker_info[0].vertex[3][1]);
	   int highx = max(max(max(marker_info[0].vertex[0][0],marker_info[0].vertex[1][0]),marker_info[0].vertex[2][0]),marker_info[0].vertex[3][0]);
	   int highy = max(max(max(marker_info[0].vertex[0][1],marker_info[0].vertex[1][1]),marker_info[0].vertex[2][1]),marker_info[0].vertex[3][1]);
	   IplImage* markerImg = cvCreateImage(cvSize(highx-lowx,highy-lowy),IPL_DEPTH_8U, 1);
	   	   IplImage* markerImgb = cvCreateImage(cvSize((highx-lowx)*2,(highy-lowy)*2),IPL_DEPTH_8U, 1);
	   	   IplImage* markerImgc = cvCreateImage(cvSize((highx-lowx)*2,(highy-lowy)*2),IPL_DEPTH_8U, 1);
	   for (int i = lowx; i < highx; i++) {
		   	   for (int j = lowy; j < highy; j++) {
				   CvScalar s= cvGet2D(adaptiveImg,j,i);
				   cvSet2D(markerImg,(j-lowy),(i-lowx),s);
			   }
	   }
	   float xdir[2], ydir[2];
	   float xlen, ylen;
	   xlen = sqrt(xaxis[0]*xaxis[0] + xaxis[1]*xaxis[1]);
	   ylen = sqrt(yaxis[0]*yaxis[0] + yaxis[1]*yaxis[1]);
	   xdir[0] = (xaxis[0]*1)/xlen;
	   xdir[1] = (xaxis[1]*1)/xlen;
	   ydir[0] = (yaxis[0]*1)/ylen;
	   ydir[1] = (yaxis[1]*1)/ylen;
	   float det = xdir[0]*ydir[1]-xdir[1]*ydir[0];
	   cout << xdir[0] << ' ' << xdir[1] << ' ' << ydir[0] << ' ' << ydir[1] << endl;
	   float transform[4] = {ydir[1]/det,-ydir[0]/det,-xdir[1]/det,xdir[0]/det};
	   cout << transform[0] << ' ' << transform[1] << ' ' << transform[2] << ' ' << transform[3] << endl;
	   int xbegin =  marker_info[0].vertex[0][0]-lowx;
	   int ybegin =  marker_info[0].vertex[0][1]-lowy;
	   float corners[4][2] = {{0,0},{transform[0]*(marker_info[0].vertex[1][0]-marker_info[0].vertex[0][0])+transform[1]*(marker_info[0].vertex[1][1]-marker_info[0].vertex[0][1]),
		   transform[2]*(marker_info[0].vertex[1][0]-marker_info[0].vertex[0][0])+transform[3]*(marker_info[0].vertex[1][1]-marker_info[0].vertex[0][1])},
			{transform[0]*(marker_info[0].vertex[2][0]-marker_info[0].vertex[0][0])+transform[1]*(marker_info[0].vertex[2][1]-marker_info[0].vertex[0][1]),
		   transform[2]*(marker_info[0].vertex[2][0]-marker_info[0].vertex[0][0])+transform[3]*(marker_info[0].vertex[2][1]-marker_info[0].vertex[0][1])},
			{transform[0]*(marker_info[0].vertex[3][0]-marker_info[0].vertex[0][0])+transform[1]*(marker_info[0].vertex[3][1]-marker_info[0].vertex[0][1]),
		   transform[2]*(marker_info[0].vertex[3][0]-marker_info[0].vertex[0][0])+transform[3]*(marker_info[0].vertex[3][1]-marker_info[0].vertex[0][1])}};
	   
	   CvScalar black;
	   black.val[0] = 0;
	   bool* untouched = new bool[(highx-lowx)*2*(highy-lowy)*2];
	   for (int i = 0; i < (highx-lowx)*2;i++)  {
		   for (int j = 0; j < (highy-lowy)*2;j++) {
			   untouched[i+(highx-lowx)*2*j] = true;
			   cvSet2D(markerImgb,j,i,black);
		   }
	   }
	   for (int i = 0; i < highx - lowx; i++) {
		   for (int j = 0; j < highy-lowy; j++) {
			   float xloc = i-xbegin;
			   float yloc = j-ybegin;
			   int transx = floor(transform[0]*xloc + transform[1]*yloc);
			   int transy = floor(transform[2]*xloc + transform[3]*yloc);
			   //cout << transx << ' ' << transy << endl;
			   if (transx < 0 || transx >= (highx-lowx)*2 || transy < 0 || transy >= (highy-lowy)*2)
				   continue;
			   untouched[transx+(highx-lowx)*2*transy] = false;
			   cvSet2D(markerImgb,transy,transx,cvGet2D(markerImg,j,i));
		   }
	   }
	   for (int i = 1; i < (highx-lowx);i++) {
		   for (int j = 1; j < (highy-lowy);j++) {
			   if (untouched[i+(highx-lowx)*2*j]) {
				   cvSet2D(markerImgb,j,i,cvGet2D(markerImgb,j-1,i));
			   }
		   }
	   }
	   int xdisp = corners[1][0]-corners[2][0];
	   int ydisp = corners[3][1]-corners[2][1];
	   for (int i = 0; i < (highx-lowx)*2;i++) 
		   for (int j = 0; j < (highy-lowy)*2;j++)
			   untouched[i+(highx-lowx)*2*j] = true;
	   for (int i = 0; i < corners[1][0]; i++) {
		   for (int j = 0; j < corners[3][1];j++) {
			   int destx = i+floor((float)(i*j*xdisp)/(corners[2][0]*corners[2][1]));
			   int desty = j+floor((float)(i*j*ydisp)/(corners[2][0]*corners[2][1]));//*floor((float)();
			   untouched[destx+(highx-lowx)*2*desty] = false;
			   cvSet2D(markerImgc,desty,destx,cvGet2D(markerImgb,j,i));
		   }
	   }
	   for (int i = 1; i < (highx-lowx)*1.5;i++) {
		   for (int j = 1; j < (highy-lowy)*1.5;j++) {
			   if (untouched[i+(highx-lowx)*2*j]) {
				   cvSet2D(markerImgc,j,i,cvGet2D(markerImgc,j-1,i));
			   }
		   }
	   }
	   
	   CvScalar white;
	   white.val[0] = 255;
	   cvReleaseImage(&markerImgb);
	   markerImgb = cvCreateImage(cvSize((highx-lowx),(highy-lowy)),IPL_DEPTH_8U,1);
	   for (int i = 0; i < (highx-lowx);i++) {
		   for (int j = 0; j < (highy-lowy);j++) {
			   if (i > (xlength)*7/8 || i < (highx-lowx)/8 || j < (ylength)/8)
				   cvSet2D(markerImgb,j,i,black);
			   else
					cvSet2D(markerImgb,j,i,cvGet2D(markerImgc,j,i));
		   }
	   }
	   IplImage* edges = cvCreateImage(cvSize((highx-lowx),(highy-lowy)),IPL_DEPTH_8U,1);
	   cvCanny(markerImgb,edges,10,100,3);

	   for (int a = 0; a < 2; a++) {
		   for (int i = 0; i < highx - lowx; i++) {
			   for (int j = 0; j < highy - lowy; j++) {
				    if (cvGet2D(markerImgb,j,i).val[0] == 255 && cvGet2D(markerImgb,j,i+1).val[0] == 0 && 
						cvGet2D(markerImgb,j,i-1).val[0] == 0 && cvGet2D(markerImgb,j-1,i).val[0] == 0 && cvGet2D(markerImgb,j+1,i).val[0] == 0) {
						if (cvGet2D(markerImgb,j-1,i-1).val[0] == 255 && cvGet2D(markerImgb,j+1,i-1).val[0] == 255) {
							cvSet2D(markerImgb,j,i,black);
							cvSet2D(markerImgb,j,i-1,white);
						}
						else if (cvGet2D(markerImgb,j-1,i-1).val[0] == 255 && cvGet2D(markerImgb,j-1,i+1).val[0] == 255) {
							cvSet2D(markerImgb,j,i,black);
							cvSet2D(markerImgb,j-1,i,white);
						}
						else if (cvGet2D(markerImgb,j+1,i+1).val[0] == 255 && cvGet2D(markerImgb,j-1,i+1).val[0] == 255) {
							cvSet2D(markerImgb,j,i,black);
							cvSet2D(markerImgb,j,i+1,white);
						}
						else if (cvGet2D(markerImgb,j+1,i+1).val[0] == 255 && cvGet2D(markerImgb,j+1,i-1).val[0] == 255) {
							cvSet2D(markerImgb,j,i,black);
							cvSet2D(markerImgb,j+1,i,white);
						}
					}
			   }
		   }
	   }
	   vector<int> iedge,jedge;
	   for (int i = 2; i < highx - lowx-2; i++) {
		   for (int j = 2; j < highy - lowy-2; j++) {
			   if (cvGet2D(markerImgb,j,i).val[0] == 0)
				   continue;
			   if (isHEdge(markerImgb,i,j,3,1) || isHEdge(markerImgb,i,j,3,-1)) {
				   if (jedge.size() == 0 || i-jedge.back() > 2)
					   jedge.push_back(i);
			   }
		   }
	   }
	   for (int j = 2; j < highy - lowy-2; j++) {
		   for (int i = 2; i < highx - lowx-2; i++) {
			   if (cvGet2D(markerImgb,j,i).val[0] == 0)
				   continue;
			   if (isVEdge(markerImgb,i,j,3,1) || isVEdge(markerImgb,i,j,3,-1)) {
				   if (iedge.size() == 0 || j-iedge.back() > 2)
					   iedge.push_back(j);
			   }
		   }
	   }

	   sort(iedge.begin(),iedge.end());
	   sort(jedge.begin(),jedge.end());
	   
	   int cutoff = 3;
	   while(iedge.size() > 7) {
		   for (int i = 0; i < iedge.size()-1;i++) {
			   if (iedge[i+1]-iedge[i] < cutoff)
				   iedge.erase(iedge.begin()+i+1);
			   if (iedge.size() == 7)
				   break;
		   }
		   cutoff++;
	   }
	   
		   cutoff = 3;
	   while(jedge.size() > 7) {
		   for (int i = 0; i < jedge.size()-1;i++) {
			   if (jedge[i+1]-jedge[i] < cutoff)
				   jedge.erase(jedge.begin()+i+1);
			   if (jedge.size() == 7)
				   break;
		   }
		   cutoff++;
	   }

	   cout << marker_info[0].id << endl;
	   if (jedge.size() != 7 || iedge.size() != 7)
		   return;
	   float out[36];
	   for (int i = 0; i < 6; i++) {
		   for (int j = 0; j < 6; j++) {
			   int is = iedge[i];
			   int js = jedge[j];
			   int ie = iedge[i+1];
			   int je = jedge[j+1];
			   out[i+6*j] = 0;
			   for (int a = is; a < ie; a++) {
				   for (int b = js; b < je; b++) {
					   out[i+6*j] += cvGet2D(markerImgc,b,a).val[0];
				   }
			   }
			   out[i+6*j] /= (float)(ie-is)*(je-js);
			   cout << out[i+6*j] << ' ';
		   }
		   cout << endl;
	   }
	   cout << xlength << ' ' << ylength << endl;
	   
     cvShowImage( "mywindow", markerImgb);
     cvShowImage( "mywindowb", edges);
	 cvWaitKey(0);

	system("pause");
   
   cvReleaseCapture( &capture );

   cvDestroyWindow( "mywindow" );
}
 
 
int main(int argc, char *argv[]) {
	while(1)
		ARstuff();

	return 0;
}