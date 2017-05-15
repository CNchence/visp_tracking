#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpOpenCVGrabber.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vpKltOpencv.h>
#include <visp/vpMbKltTracker.h>
#include <visp/vpMbEdgeKltTracker.h>

using namespace std;

int main()
{
	string object, method;


	//object = "白盒子";
	//object = "圆柱体";
	object = "zeiss";

	//method = "model_based";
	method = "model+keypoints";





// // SOURCE FROM CAMERA
vpImage<unsigned char> I;
cv::Mat Frame;
//Image grabber initialisation

cv::VideoCapture capture(0);//如果是笔记本，0打开的是自带的摄像头，1 打开外接的相机  
//double rate = 25.0;//视频的帧率

// SOURCE FROM IMAGE
//vpImage<unsigned char> I;
//vpImageIo::read(I,"gbox.pgm");

//Display initialisation
#if defined VISP_HAVE_X11
vpDisplayX d;
//#elif defined VISP_HAVE_GDI
//vpDisplayGDI d;
#elif defined VISP_HAVE_OPENCV
vpDisplayOpenCV d;
#endif

capture >> Frame;
cout << "图片大小:" << endl;
cout << Frame.cols;
cout << Frame.rows << endl;
vpImageConvert::convert(Frame,I);


d.init(I, 0, 0, "") ;

vpDisplay::display(I);
vpDisplay::flush(I);
vpMbTracker *tracker;
if (method == "model_based")
tracker = new vpMbEdgeTracker;
else if (method == "model+keypoints")
tracker = new vpMbEdgeKltTracker;
vpHomogeneousMatrix cMo;

//cout << endl << "333333333" << endl;
//cin >> c;
//vpCameraParameters cam(839, 839, 325, 243);
//vpCameraParameters cam(1920, 1080, 922, 523);
vpCameraParameters cam(839, 839, 307, 232);

vpMe me;
//vpCameraParameters cam(383, 382, 162, 118); // CAM PARAMETERS... SEE *.XML FILE
//vpCameraParameters cam(1920, 1080, 960, 540); // CAM PARAMETERS... SEE *.XML FILE
//vpCameraParameters cam(640,480,511,511); // CAM PARAMETERS... SEE *.XML FILE

if (object == "圆柱体")
{
	//圆柱体(茶叶)
	tracker->setCameraParameters(cam);
	me.setMaskSize(5); // ecm:mask:size
	me.setMaskNumber(180); // ecm:mask:nb_mask
	me.setRange(8); // ecm:range:tracking
	me.setThreshold(2000); // ecm:contrast:edge_threshold
	me.setMu1(0.5); // ecm:contrast:mu1
	me.setMu2(0.5); // ecm:contrast:mu2
	me.setSampleStep(4); // sample:step
	dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(me);
	if (method == "model+keypoints")
	{
		vpKltOpencv klt_settings;
		klt_settings.setMaxFeatures(10);
		klt_settings.setWindowSize(5);
		klt_settings.setQuality(0.015);
		klt_settings.setMinDistance(8);
		klt_settings.setHarrisFreeParameter(0.01);
		klt_settings.setBlockSize(3);
		klt_settings.setPyramidLevels(3);
		dynamic_cast<vpMbKltTracker*>(tracker)->setKltOpencv(klt_settings);
		dynamic_cast<vpMbKltTracker*>(tracker)->setMaskBorder(5);
	}
	tracker->loadModel("cy.cao");
	tracker->initClick(I, "cy.init", false);
	tracker->savePose("cy.0.pos");


}
else if (object == "白盒子")
{
	//白盒子
	tracker->setCameraParameters(cam);
	me.setMaskSize(5); // ecm:mask:size
	me.setMaskNumber(180); // ecm:mask:nb_mask
	me.setRange(8); // ecm:range:tracking
	me.setThreshold(10000); // ecm:contrast:edge_threshold
	me.setMu1(0.5); // ecm:contrast:mu1
	me.setMu2(0.5); // ecm:contrast:mu2
	me.setSampleStep(4); // sample:step
	dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(me);
	if (method == "model+keypoints")
	{
		vpKltOpencv klt_settings;
		klt_settings.setMaxFeatures(100);
		klt_settings.setWindowSize(5);
		klt_settings.setQuality(0.015);
		klt_settings.setMinDistance(8);
		klt_settings.setHarrisFreeParameter(0.01);
		klt_settings.setBlockSize(3);
		klt_settings.setPyramidLevels(3);
		dynamic_cast<vpMbKltTracker*>(tracker)->setKltOpencv(klt_settings);
		dynamic_cast<vpMbKltTracker*>(tracker)->setMaskBorder(5);
	}
	tracker->loadModel("teabox.cao");
	tracker->initClick(I, "teabox.init", false);
	tracker->savePose("teabox.0.pos");

}
else if (object == "zeiss")
{
	tracker->setCameraParameters(cam);
	me.setMaskSize(5); // ecm:mask:size
	me.setMaskNumber(180); // ecm:mask:nb_mask
	me.setRange(8); // ecm:range:tracking
	me.setThreshold(5000); // ecm:contrast:edge_threshold
	me.setMu1(0.5); // ecm:contrast:mu1
	me.setMu2(0.5); // ecm:contrast:mu2
	me.setSampleStep(4); // sample:step
	dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(me);
	if (method == "model+keypoints")
	{
		vpKltOpencv klt_settings;
		klt_settings.setMaxFeatures(30);
		klt_settings.setWindowSize(5);
		klt_settings.setQuality(0.015);
		klt_settings.setMinDistance(8);
		klt_settings.setHarrisFreeParameter(0.01);
		klt_settings.setBlockSize(3);
		klt_settings.setPyramidLevels(1);
		dynamic_cast<vpMbKltTracker*>(tracker)->setKltOpencv(klt_settings);
		dynamic_cast<vpMbKltTracker*>(tracker)->setMaskBorder(5);
	}
	tracker->loadModel("zeiss.cao");
	tracker->initClick(I, "zeiss.init", false);
	tracker->savePose("zeiss.0.pos");
}


//me.setNbTotalSample(600); // sample:nb_sample

// tracker.setMovingEdge(me); // Initialise the moving edges according to a guide


// track the model
/*
tracker->track(I);
tracker->getPose(cMo);
tracker->display(I, cMo, cam, vpColor::red, 1);
vpDisplay::flush(I);*/




//getchar();

while ( 1 )
{

	capture >> Frame;                                           //opencv获取每一帧
	vpImageConvert::convert(Frame, I);                        //将opencv的一帧转为vpimage
	string key;
	string MaskSize;
	//g.acquire(I); // CAMERA
		/*
	vpImageIo::read(I,"gbox.pgm"); // IMAGE
	vpDisplay::display(I);
	*/
	//Track the object
	vpDisplay::display(I);
	tracker->track(I);
	tracker->getPose(cMo);
	tracker->getCameraParameters(cam);
	tracker->setDisplayFeatures(true);
	tracker->display(I, cMo, cam, vpColor::red, 1);
	vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
	//vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::blue);

	/*
	if (vpDisplay::getKeyboardEvent(I, key, false))
	{
		if (key == "97")
		{
			//me.setMaskSize(me.getMaskSize() + 1);
			me.setMaskSize(50);
			//dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(me);
		}
		if (key == "98")
		{
			me.setMaskSize(me.getMaskSize() - 1);
			//dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(me);
		}
		cout << key << endl;

		//dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(me);
		MaskSize = to_string(me.getMaskSize());

		cout << "masksize==" << MaskSize << endl;

		
	}
	*/
	//vpDisplay::displayText(I, 10, 10, "vpMaskSize = "+ MaskSize, vpColor::red);

	/*
	me.setMaskSize(5); // ecm:mask:size
	me.setMaskNumber(180); // ecm:mask:nb_mask
	me.setRange(8); // ecm:range:tracking
	me.setThreshold(5000); // ecm:contrast:edge_threshold
	me.setMu1(0.5); // ecm:contrast:mu1
	me.setMu2(0.5); // ecm:contrast:mu2
	me.setSampleStep(5); // sample:step
	*/

	vpDisplay::flush(I);
}

return 0;
}