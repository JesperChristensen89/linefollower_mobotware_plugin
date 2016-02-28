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
  UART uart;
 
  /**
  Constructor */
  UFuncLineFollower()
  {
    setCommand("linefollower", "linefollower", "camera base line follower (compiled " __DATE__ " " __TIME__ ")");
    // create global variables
    createBaseVar();
    	


    uart.init();
    
    uart.send((char *)"M=8\n");
    uart.send((char *)"start\n");
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
    bool stopRunning;
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
      stopRunning = msg->tag.getAttValue("stop",val,MVL);
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
    else if (stopRunning)
    {
      UART u;
      u.send((char *)"998\n");
      
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
	

	
	img->toBW(img);
    
	cv::Mat imgCV = cv::cvarrToMat(img->cvArr());
	
	linefollower.start(imgCV,left,white,errThresh);
	
	//uart.send((char *)"998\n");
	
	
	/*
        //result = findBall(cam, img, debug, gotBlue);
        if (smrcl)
        { // format for MRC
          snprintf(reply, MRL, "<vision vis1=\"%d\" vis2=\"%g\" vis3=\"%g\" vis4=\"%d\"/>\n",
                                         result, getPos().x, getPos().y, getCnt());
          sendMsg(msg, reply);
        }
        else if (not result)
        { // did not find any balls in image - with a reasonable size
          snprintf(reply, MRL, "<%s cnt=\"%d\"/>\n", msg->tag.getTagName(), getCnt());
          sendMsg(msg, reply);
        }
        else
        { // send XML open tag with ball count as attribute
          snprintf(reply, MRL, "<%s cnt=\"%d\">\n", msg->tag.getTagName(), getCnt());
          sendMsg(msg, reply);
          // code position of the ball and send
          tag.codePosition(getPos(), reply, MRL, "ball");
          result = sendMsg(msg, reply);
          // send XML close tag
          snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
          sendMsg(msg, reply);
        }
        */
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
    varPoolImg = addVar("poolImg", 45.0, "d", "(r/w) first image pool number to use");
  }
  

  //
private:
  
  int imgSizeH;
  /// size of source image in pixels
  int imgSizeW;
  /// pointer to limiting red values redMin, redMax, greenMin, greenMax
  UVariable * varPoolImg;
  /// pointer to limiting red values redMin, redMax, greenMin, greenMax
};


#ifdef LIBRARY_OPEN_NEEDED
UFunctionBase * createFunc()
{ // called by server to create an object of this type
  /** replace 'UFuncLineFollower' with your classname, as used in the headerfile */
  return new UFuncLineFollower();
}
#endif
