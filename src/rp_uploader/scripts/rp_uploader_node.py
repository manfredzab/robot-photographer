#!/usr/bin/env python

# @class      RPUploaderNode
#
# @brief      Robot photographer's node which uploads taken pictures to the Flickr online
#             gallery, using the Flickr API. 
#
# @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
#             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.

# ROS imports
import roslib; roslib.load_manifest("rp_uploader")
import rospy
from std_msgs.msg import String

# Flickr API imports
import flickrapi
from flickrapi.exceptions import FlickrError

# RPUploader imports
from rp_uploader.srv import UploaderService
from rp_uploader.srv import UploaderServiceRequest
from rp_uploader.srv import UploaderServiceResponse

class RPUploaderNode():        
               
    # Constructor
    def __init__(self):               
        # Read the Flickr API parameters from the parameter server 
        self.get_flickr_api_parameters()
        
        # Initialize the connection to Flickr
        self.connect_to_flickr()
        
        # Initialize the picture filename subscriber
        self.service = rospy.Service("/rp/uploader/upload", UploaderService, self.photo_upload_service_callback)      
        
        # Spin at 5 Hz
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()
            
    
    # Sets up the connection to Flickr (API parameters must be initialized before this call)
    def connect_to_flickr(self):
        try:
            self.flickr = flickrapi.FlickrAPI(self.api_key, self.api_secret)
            (token, frob) = self.flickr.get_token_part_one(perms = "write")
        
            # A browser window will be opened to ask for write permissions on Flickr
            if not token:
                raw_input("Press ENTER after you gave permissions for Robot Photographer...")
        
                self.flickr.get_token_part_two((token, frob))
        
        except FlickrError:
            rospy.logerr("Could not open the Flickr connection! Check whether the API key and secret parameters are set correctly.")     
       
       
    # Uploads a picture with a given filename to Flickr
    def photo_upload_service_callback(self, upload_request):
        # Get the filename from the service request
        input_filename = upload_request.picture_file_name
        
        # Get parameters from the parameter server (photo title, description and tags)
        self.get_photo_parameters()        
        
        try:           
            # Upload the photo to Flickr            
            self.flickr.upload(filename = input_filename, 
                               title = self.photo_title,
                               description = self.photo_description,
                               tags = self.photo_tags,
                               callback = self.photo_upload_callback)
            
            return UploaderServiceResponse(True)
        
        except IOError:
            rospy.logerr("Could open picture %s." % input_filename)            
            return UploaderServiceResponse(False)
            
        except FlickrError:
            rospy.logerr("Could not upload picture %s to Flickr." % input_filename)            
            return UploaderServiceResponse(False)
                
       
    # Returns Flickr API parameters from the parameter server
    def get_flickr_api_parameters(self):
        self.api_key = rospy.get_param("/rp/uploader_node/flickr_api_key")
        self.api_secret = rospy.get_param("/rp/uploader_node/flickr_api_secret")
               
        
    # Returns photo parameters from the parameter server
    def get_photo_parameters(self):
        self.photo_title = rospy.get_param("/rp/uploader_node/photo_title")
        self.photo_description = rospy.get_param("/rp/uploader_node/photo_description")
        self.photo_tags = rospy.get_param("/rp/uploader_node/photo_tags")


    # Empty callback for the photo upload progress (irrelevant for a synchronous call)
    def photo_upload_callback(self, progress, done):
        pass # Do nothing
    
    
if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("rp_uploader")
    
    # Create the worker service
    try:
        node = RPUploaderNode()
        
    except rospy.ROSInterruptException:
        pass # Do nothing
