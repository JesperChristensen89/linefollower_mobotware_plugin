/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
 *   See svn log for modification history                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef OPENCV2
#include <legacy/compat.hpp>
#endif

#include <urob4/usmltag.h>
#include <cstdlib>
#include <boost/concept_check.hpp>
#include <ucam4/ufunctioncambase.h>
#include "../aupoly/urespoly.h"
#include "linefollower.h"
#include "uart.h"
#include <urob4/uresposehist.h>
//#include <../libs/eigen3/Eigen/src/Eigen2Support/Geometry/Translation.h>
#include <highgui.h>
#include <time.h>
#include "opencv2/imgproc/imgproc.hpp"
//#include <opencv2/imgcodecs.hpp>
#include <pthread.h>

using namespace cv;

UART uart;
bool isRegBotRunning;

  
void *uartReceiver(void * threadid)
{

  while (isRegBotRunning)
  {
    uart.receive();
  }

  pthread_exit(NULL);
  
    
}


/**
Vision Based Line Follower designed for regbots
@author Jesper H. Christensen
*/
class UFuncLineFollower : public UFunctionCamBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunc followed by
  // a descriptive extension for this specific plugin

  
public:
  
  LineFollower linefollower;

 
  
  
  /**
  Constructor */
  UFuncLineFollower()
  {
    setCommand("linefollower", "linefollower", "camera base line follower (compiled " __DATE__ " " __TIME__ ")");
    // create global variables
    createBaseVar();
    	
    uart.init();
    
    uart.send((char *)"M=8\n");
    
    int threadOK;
    
    pthread_t receiverThread;
    threadOK = pthread_create(&receiverThread, NULL, uartReceiver, (void*)&receiverThread);
    if (threadOK != 0)
    {
      exit(EXIT_FAILURE);
    }
    
  };
  /**
  Destructor - to delete the resource (etc) when finished */
  virtual ~UFuncLineFollower()
  { // possibly remove allocated variables here - if needed
  }
  /**
  Handle incomming command
  Must return true if the function is handled -
  otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra)
  { // handle command(s) send to this plug-in
    const int MRL = 2000;
    char reply[MRL];
    bool ask4help;
    const int MVL = 50;
    char val[MVL];
    int camDevice = -1;
    bool imgPoolIsSet = false;
    int imgPoolNum = -1;
    UImage * img = (UImage *)extra;
    USmlTag tag;
    UCamPush * cam = NULL;
    bool result;
    bool debug = true; // default is debug on
    bool smrcl = false; // default is <ball ...> reply
    bool white = false; // default is black
    bool left  = false; // default is right
    int errThresh = 4; 
    // check for parameters - one parameter is tested for - 'help'
    // the help value is ignored, e.g. if help="bark", then
    // the value "bark" will be in the 'helpValue' string.
    ask4help = msg->tag.getAttValue("help", val, MVL);
    if (not ask4help)
    { // get all other parameters
      msg->tag.getAttValueInt("device", &camDevice);
      imgPoolIsSet = msg->tag.getAttValueInt("img", &imgPoolNum);
      msg->tag.getAttValueBool("white", &white, true);
      msg->tag.getAttValueBool("left", &left, true);
      msg->tag.getAttValueInt("error",&errThresh);
      msg->tag.getAttValueBool("debug", &debug, true);
      msg->tag.getAttValueBool("smrcl", &smrcl, true);
    }
    // ask4help = false, if no 'help' option were available.
    if (ask4help)
    { // create the reply in XML-like (html - like) format
      sendHelpStart("LineFollower");
      sendText("--- available LineFollower options\n");
      sendText("device=X          Use this camera - for position and parameters\n");
      sendText("img=X             Get image from image pool - else take new image\n");
      sendText("white             Will find white lines (def is black)\n");
      sendText("left		  Will make left turns (def is right)\n");
      sendText("error=X		  sets a threshold on how many errors the robot can make before stopping - if it fails during turns, this variable probably needs to go up! [default = 4]\n");
      sendText("smrcl             Format the reply for MRC (<vision vis1=\"x.x\" vis2=\"y.y\" .../>\n");
      sendText("help              This message\n");
      sendText("See also: var linefollower for other parameters and results\n");
      sendHelpDone();
      sendInfo("done");
      result = true;
     
    }
    else
    { 
      
	
	if (imgPoolIsSet)
        { // take image from image pool
          img = imgPool->getImage(imgPoolNum, false);
          result = (img != NULL);
          if (result and camDevice < 0)
            //take image device from image
            camDevice = img->camDevice;
          if (result)
          {
            cam = camPool->getCam(camDevice);
            result = (cam != NULL);
          }
        }
	else if (img != NULL)
      { // we have an image, so just camera is needed
        camDevice = img->camDevice;
        cam = camPool->getCam(camDevice);
        result = (cam != NULL);
      }
      else
      { // get new image from a camera and store in first imagepool number for ball images
        img = imgPool->getImage(varPoolImg->getInt(), true);
        result = getCamAndRawImage(&cam,        // result camera           out
                                  &img,        // result image            out
                                  &camDevice,  // camera device number    in-out
                                  NULL,        // pushed image (YUV)      in
                                  "", camDevice + 3);         // camera position name    in
        if (result)
          result = (img != NULL and cam != NULL);
      }
      // camera and image is now available
      // time to kick some ass
      if (result)
      { // there is an image, make the required ball analysis
	

	if (uart.getMissionStart())
	{
	    
	  img->toBW(img);
      
	  cv::Mat imgCV = cv::cvarrToMat(img->cvArr());
	  
	  isRegBotRunning = linefollower.start(imgCV, uart, left,white,errThresh);
	}
	  
	//uart.send((char *)"998\n");
	
      }
      else
      {
        snprintf(reply, MRL, "failed, got image %s, got camera %d %s\n",
                bool2str(img != NULL), camDevice, bool2str(cam != NULL));
        sendWarning(msg, reply);
      }
    }


    // return true if the function is handled with a positive result
    return result;
  }
  
  
  

 

protected:



  /**
  Make the variables that will be available to other plugins */
  void createBaseVar()
  {
    varPoolImg  = addVar("poolImg", 45.0, "d", "(r/w) first image pool number to use");
    //varLeft     = addVar("left", false, "b", "turn direction");
    //varWhite    = addVar("white", false, "b", "Line color to follow");
    //varErrThresh= addVar("errThresh", 4, "i", "Errors to make before exiting");
    
  }
  

  //
private:
  
  int imgSizeH;
  /// size of source image in pixels
  int imgSizeW;

  UVariable * varPoolImg;
  UVariable * varLeft;
  UVariable * varWhite;
  UVariable * varErrThresh;
};


#ifdef LIBRARY_OPEN_NEEDED
UFunctionBase * createFunc()
{ // called by server to create an object of this type
  /** replace 'UFuncLineFollower' with your classname, as used in the headerfile */
  return new UFuncLineFollower();
}
#endif
