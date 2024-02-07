#!/usr/bin/env python3
import re
import rospy
from sendgrid import SendGridAPIClient
from sendgrid.helpers.mail import *
from rcomponent.rcomponent import RComponent
from robotnik_alarms_msgs.srv import SendAlarms, SendAlarmsResponse, SendAlarmsRequest


class SimpleEmailSender(RComponent):

    def __init__(self):
        RComponent.__init__(self)

    def ros_read_params(self) -> None:
        """"""
        RComponent.ros_read_params(self)

        self.sender =  rospy.get_param('sendgrid/sender', 'sender@domain.com')
        self.default_recipients = rospy.get_param('sendgrid/default_recipients', 'recipient@domain.com')
        self.time_between_emails = rospy.get_param('sendgrid/time_between_emails', 0)
        self.sendgrid_api_key = rospy.get_param('sendgrid/api_key')

    def ros_setup(self) -> None:
        """"""
        RComponent.ros_setup(self)

        self.send_email_service = rospy.Service('robotnik_email/send_email', SendAlarms, self.send_email_cb)

    def init_state(self) -> None:
        """
        """
        if self.check_recipients(self.default_recipients) == False:
            rospy.logerr("Default recipients are malformed")
            rospy.signal_shutdown("shutdown")

        if self.check_recipients(self.sender.split()) == False:
            rospy.logerr("Sender is malformed")
            rospy.signal_shutdown("shutdown")

        return RComponent.init_state(self)
    
    def send_email_cb(self, request: SendAlarmsRequest) -> None:
        """"""
        response = SendAlarmsResponse()
        response.ret.success = False
        response.ret.code = -1

        email_html = self.build_email(request)
        sendgrid_msg = Mail()
        sendgrid_msg.to = [
            To(
                email=recipient,
                name='Recipient',
                p=0                       
        ) for recipient in self.default_recipients]
        sendgrid_msg.from_email = From(
            email=self.sender,
            name="Alarm system",
            p=1
        )
        sendgrid_msg.reply_to = ReplyTo(
            email=self.sender,
            name="Alarm system"
        )
        sendgrid_msg.subject = Subject('Alarm')
        sendgrid_msg.content = [
            Content(
                mime_type='text/html',
                content=email_html
            )
        ]
        sendgrid_msg.category = [Category('Alarm')]

        try:
            sg = SendGridAPIClient(self.sendgrid_api_key)
            sg_response = sg.send(sendgrid_msg)
            rospy.loginfo(f'Email sending request response: [{sg_response.status_code}]')
            
            if str(sg_response.status_code)[:2] == '20':
                response.ret.success = True
                response.ret.code = 0
                return response
            
            return response
        except Exception as err:
            rospy.loginfo(err.message)
            return response

    def check_recipients(self, recipients):

        regex = '^(\w|\.|\_|\-)+[@](\w|\_|\-|\.)+[.]\w{2,3}$'
        valid = True

        for recipient in recipients:

            if not re.search(regex, recipient):

                rospy.logerr("%s is an invalid email", recipient)
                valid = False

        return valid
    
    def build_email(self, email_data):

        email = f'<p>Hello from Robotnik Alarm System,</p><p>An alarm of type {email_data.status.type} was triggered at time {email_data.datetime}.</p><p>Description of the alarm: {email_data.status.description}.</p>'

        return email
