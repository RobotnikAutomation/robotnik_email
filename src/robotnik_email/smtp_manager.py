#!/usr/bin/env python

# https://www.youtube.com/watch?v=ql5Dex4m40w
# https://www.gmass.co/smtp-test

import rospy 

from rcomponent.rcomponent import *
from robotnik_msgs.srv import SetString
from robotnik_alarms_msgs.srv import SendAlarms, SendAlarmsResponse

import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import re

class SMTPManager(RComponent):
    """
    SMTP server to send messages through ROS
    """

    def __init__(self):

        RComponent.__init__(self)
        
        self.logger = self.initialize_logger()

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.smtp_server = rospy.get_param('smtp/server', 'smtp.gmail.com')
        self.smtp_port = rospy.get_param('smtp/port', 587)
        self.sender =  rospy.get_param('smtp/sender', 'sender@domain.com')
        self.use_authentication = rospy.get_param('smtp/use_authentication', False)
        self.username = rospy.get_param('smtp/username', 'username')
        self.password = rospy.get_param('smtp/password', 'password')
        self.default_recipients = rospy.get_param('smtp/default_recipients', 'recipient@domain.com')
        self.time_between_emails = rospy.get_param('smtp/time_between_emails', 0)


    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Service
        self.send_email_service = rospy.Service('robotnik_email/send_email', SendAlarms, self.send_email_cb)
        
        return 0

    def init_state(self):
        
        if self.check_recipients(self.default_recipients) == False:
            self.logger.logerror("Default recipients are malformed", "")
            rospy.signal_shutdown("shutdown")

        if self.check_recipients(self.sender.split()) == False:
            self.logger.logerror("Sender is malformed", "")
            rospy.signal_shutdown("shutdown")

        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        return RComponent.ready_state(self)

    def emergency_state(self):
        
        dummy = 0
        #self.switch_to_state(State.READY_STATE)

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """
        try:
            self.smtp_disconnection()
        except:
            pass

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def check_recipients(self, recipients):

        regex = '^(\w|\.|\_|\-)+[@](\w|\_|\-|\.)+[.]\w{2,3}$'
        valid = True

        for recipient in recipients:

            if not re.search(regex, recipient):

                self.logger.logerror(f"{recipient} is an invalid email", "")
                valid = False

        return valid


    def send_email_cb(self, req):

        response = SendAlarmsResponse()
        response.ret.success = False
        response.ret.code = -1

        if self.smtp_connection():

            email = self.build_email(req)

            if email != None:

                if self.send_email(email):
                    
                    #response.ret.message = "Email sent from " + email["From"] + " to " + email["To"]
                    self.logger.loginfo("Email sent from " + email["From"] + " to " + email["To"], "")
                    response.ret.success = True
                    response.ret.code = 0
                    
                else:

                    response.ret.message = "The email could not be sent to the recipients, probably due to a mail server failure"

            else:
                response.ret.message = "The email can not be sent because it is malformed"
        else:

            response.ret.message = "Cannot connect to SMTP server " + str(self.smtp_server) + " with port " + str(self.smtp_port)

        try:
            self.smtp_disconnection()
        except:
            pass

        #if (response.ret.success == True) or (response.ret.code == 0):
        #    rospy.loginfo(response.ret.message)
        if (response.ret.success == False) or (response.ret.code == -1):
            self.logger.logerror(response.ret.message, "")

        return response


    def smtp_connection(self):

        # self.smtp = smtplib.SMTP(timeout=20)
        self.smtp = smtplib.SMTP(self.smtp_server)
        
        try:
            self.smtp.connect(self.smtp_server, self.smtp_port)
            self.smtp.ehlo()
            self.smtp.starttls()
            self.smtp.ehlo()

            if self.use_authentication:
                self.smtp.login(self.username, self.password)

            success = True

        except Exception as e:
            self.logger.logerror(f"smtp_manager::smtp_connection -> Exception: {e}", "")
            success = False

        return success


    def build_email(self, email_data):

        email = MIMEMultipart("alternative")
  
        email["From"] = self.sender
        email["Subject"] =  "default"
        email.attach(MIMEText(email_data.status.message, "html"))

        if email_data.status.message == "":
            self.logger.logewarning("Message email is empty", "")

        if '' in email_data.recipients or len(email_data.recipients) == 0:
            email["To"] = ', '.join(self.default_recipients)

        else:
            if self.check_recipients(email_data.recipients):
                email["To"] = ', '.join(email_data.recipients)
            else:
                email = None

        return email

    def send_email(self, email):

        try: 
            #print(email["To"])
            self.smtp.sendmail(email["From"], email["To"].split(','), email.as_string())
            success = True

        except Exception as e:
            self.logger.logerror(f"smtp_manager::send_email -> Exception: {e}", "")
            success = False
        
        return success

    def smtp_disconnection(self):

        self.smtp.quit()