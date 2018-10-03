#include "yolo_v2_class.hpp"

#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char *argv[])
{   
    if(argc == 3)
    {
        char *cfg     = argv[1];
        char *weights = argv[2];

        Detector detector(cfg, weights, 0);

        std::cout << "Network Width: " << detector.get_net_width() << "\n" << "Network Height: " << detector.get_net_height();

        detector.detect("image07609.jpg");
        
        printf("YO MAN");
    }
}

/*
OLD MAIN from darknet_wrapper


#include "wrapper/wrapper_cpp.h"
#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>

#include <pylon/PylonIncludes.h>

//Declared in darknet/src/wrapper_c.h
network_configuration make_config()
{
    network_configuration config;

    config.cfgfile    = "test/tiny-yolo-voc.cfg";
    config.names_file = "data/voc.names";
    config.weightfile = "test/tiny-yolo-voc.weights";

    return config;
}

int main(int argc, char *argv[]){

    network_configuration config = make_config();
    //char *default_image = "test/dog.jpg";
    char *test_image = "test/image07609.jpg";
    float thresh = 0.2;
    int hits;
    box **outboxes = new box*;
    float **outprobs = new float*;
    int **outclasses = new int*;


    Darknet::getInstance(config.names_file, config.cfgfile, config.weightfile);

    /*
    Darknet::getInstance().darknet_detect_file(test_image, thresh, &hits, outboxes, outprobs, outclasses);
    
    cv::Mat img  = cv::imread(default_image, cv::IMREAD_UNCHANGED);
    IplImage ipl = img;

    Darknet::getInstance().darknet_detect_img(&ipl, thresh, &hits, outboxes, outprobs, outclasses); 


    free((*outboxes));
    free((*outprobs));
    free((*outclasses));

    */  /*
    int exitCode = 0;

    Pylon::PylonAutoInitTerm autoInitTerm;

    // apply the detector to an image file
    //char *filename = "test/image07609.jpg";

    try
    {
        std::cout << "Creating Camera" << std::endl;
        Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());

        std::cout << "Camera Created.\n" << "Using Device: " << camera.GetDeviceInfo().GetModelName() << std::endl;

        //Possible Parameter to play with - Default is 10
        camera.MaxNumBuffer = 10;

        Pylon::CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;

        Pylon::CPylonImage pylonImage;

        cv::Mat cvMat;

        //We only need the latest image for Yolo
        camera.StartGrabbing(Pylon::EGrabStrategy::GrabStrategy_LatestImageOnly, Pylon::EGrabLoop::GrabLoop_ProvidedByUser);

        Pylon::CGrabResultPtr ptrGrabResult;

        while( camera.IsGrabbing())
        {
            //Wait on input to Grab Image and run prediction
            /*std::cerr << "Press Enter to Grab and Predict an Image." << std::endl;
            if( std::cin.get() == '\n')
            {*/ /*
                camera.RetrieveResult( 5000, ptrGrabResult, Pylon::ETimeoutHandling::TimeoutHandling_ThrowException);

                if( ptrGrabResult->GrabSucceeded())
                {
                    // Convert the grabbed buffer to pylon imag
                    formatConverter.Convert(pylonImage, ptrGrabResult);
                    // Create an OpenCV image out of pylon image
                    cvMat = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

                    IplImage* image;
                    image = cvCreateImage(cvSize(cvMat.cols, cvMat.rows), CV_8U, 3);
                    IplImage ipltemp = cvMat;
                    cvCopy(&ipltemp, image);

                    Darknet::getInstance().darknet_detect_img(image, thresh, &hits, outboxes, outprobs, outclasses);

                    std::cout << "hits: " << hits << "\n";
                    for (int i = 0; i < hits; i++) {
                        box hitbox = (*outboxes)[i];
                        int label = (*outclasses)[i];
                        float prob = (*outprobs)[i];

                        // calculate coordinates of bounding box
                        // yolo returns the center of the bbox
                        float x, y, w, h;
                        x = hitbox.x - hitbox.w / 2;
                        y = hitbox.y - hitbox.h / 2;
                        w = hitbox.w;
                        h = hitbox.h;

                        cv::rectangle(cvMat, {x,y}, {x+w, y+h}, {0,0,255}, 2);
                    }

                    // Create a display window
                    cv::namedWindow( "OpenCV Display Window", CV_WINDOW_NORMAL);//AUTOSIZE //FREERATIO
                    // Display the current image with opencv
                    cv::imshow( "OpenCV Display Window", cvMat);
                    // Define a timeout for customer's input in ms.
                    // '0' means indefinite, i.e. the next image will be displayed after closing the window 
                    // '1' means live stream
                    cv::waitKey(1);
	
	            cvReleaseImage(&image);
	
		    std::cout << "\033[2J";
   		    std::cout << "\033[1;1H";			
	
                    free((*outboxes));
                    free((*outprobs));
                    free((*outclasses));

                }
                else
                {
                    std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
                }
            //}
        }
    }
    catch (Pylon::GenericException &e)
    {
        std::cerr << "An exception occurred." << std::endl << e.GetDescription() << std::endl;
        exitCode = 1;
    }

    return exitCode;
}

void display_thread()
{

}

void detect_thread()
{

}

void retrieve_thread(IplImage *image, Pylon::CInstantCamera camera, Pylon::CGrabResultPtr ptrGrabResult, Pylon::CImageFormatConverter formatConverter)
{
    camera.RetrieveResult( 5000, ptrGrabResult, Pylon::ETimeoutHandling::TimeoutHandling_ThrowException);

        if( ptrGrabResult->GrabSucceeded())
                {
            // Convert the grabbed buffer to pylon imag
            formatConverter.Convert(pylonImage, ptrGrabResult);
            // Create an OpenCV image out of pylon image
            cvMat = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

            image = cvCreateImage(cvSize(cvMat.cols, cvMat.rows), CV_8U, 3);
            IplImage ipltemp = cvMat;
            cvCopy(&ipltemp, image);
}*/