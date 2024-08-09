#!/usr/bin/env python

# https://www.youtube.com/watch?v=ql5Dex4m40w
# https://www.gmass.co/smtp-test

from os.path import basename
from typing import List, Union

import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.application import MIMEApplication
import re

import rospy

from rcomponent.rcomponent import RComponent
from robotnik_alarms_msgs.srv import SendAlarms, SendAlarmsResponse


class SMTPManager(RComponent):
    """
    SMTP server to send messages through ROS
    """

    def __init__(self):

        RComponent.__init__(self)
        self.smtp_server = ''
        self.smtp_port = 0
        self.sender = ''
        self.use_authentication = False
        self.username = ''
        self.password = ''
        self.default_recipients = ''
        self.time_between_emails = 0
        self.smtp = None
        self.send_email_service = None

        self.logger = self.initialize_logger()

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.smtp_server = rospy.get_param('smtp/server', 'smtp.gmail.com')
        self.smtp_port = rospy.get_param('smtp/port', 587)
        self.sender = rospy.get_param('smtp/sender', 'sender@domain.com')
        self.use_authentication = rospy.get_param(
            'smtp/use_authentication', False)
        self.username = rospy.get_param('smtp/username', 'username')
        self.password = rospy.get_param('smtp/password', 'password')
        self.default_recipients = rospy.get_param(
            'smtp/default_recipients', 'recipient@domain.com')
        self.time_between_emails = rospy.get_param(
            'smtp/time_between_emails', 0)

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Service
        self.send_email_service = rospy.Service(
            'robotnik_email/send_email', SendAlarms, self.send_email_cb)

        return 0

    def init_state(self):
        """
        Initializes the state of the SMTP manager.

        This method checks if the default recipients and sender are properly formatted.
        If any of them is malformed, it logs an error and shuts down the ROS node.

        Returns:
            The initialization state of the SMTP manager.

        """
        if self.check_recipients(self.default_recipients) is False:
            self.logger.logerror("Default recipients are malformed", "")
            rospy.signal_shutdown("shutdown")

        if self.check_recipients(self.sender.split()) is False:
            self.logger.logerror("Sender is malformed", "")
            rospy.signal_shutdown("shutdown")

        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        return RComponent.ready_state(self)

    def emergency_state(self):
        """
        Method to handle emergency state.

        This method is responsible for handling the emergency state of the SMTP manager.
        It performs necessary actions to handle the emergency situation.

        Parameters:
            self (SMTPManager): The SMTPManager instance.

        Returns:
            None
        """
        dummy = 0
        # self.switch_to_state(State.READY_STATE)

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """
        self.smtp_disconnection()

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def check_recipients(self, recipients):
        """
        Checks if the given recipients are valid email addresses.

        Args:
            recipients (list): A list of email addresses to be checked.

        Returns:
            bool: True if all recipients are valid email addresses, False otherwise.
        """

        regex = '^(\w|\.|\_|\-)+[@](\w|\_|\-|\.)+[.]\w{2,3}$'
        valid = True

        for recipient in recipients:
            if not re.search(regex, recipient):
                self.logger.logerror(f"{recipient} is an invalid email", "")
                valid = False

        return valid

    def send_email_cb(self, req):
        """
        Sends an email based on the given request.

        Args:
            req: The request object containing the email details.

        Returns:
            A response object indicating the success or failure of the email sending operation.
        """
        response = SendAlarmsResponse()
        response.ret.success = False
        response.ret.code = -1

        if self.smtp_connection():

            email = self.build_email(req)

            if email is not None:

                if self.send_email(email):

                    # response.ret.message = "Email sent from " + email["From"] + " to " + email["To"]
                    self.logger.loginfo(
                        "Email sent from " + email["From"] + " to " + email["To"], "")
                    response.ret.success = True
                    response.ret.code = 0

                else:

                    response.ret.message = "The email could not be sent to the recipients, probably due to a mail server failure"

            else:
                response.ret.message = "The email can not be sent because it is malformed"
        else:

            response.ret.message = "Cannot connect to SMTP server " + \
                str(self.smtp_server) + " with port " + str(self.smtp_port)

        try:
            self.smtp_disconnection()
        except smtplib.SMTPException as e:
            rospy.logerr(e)

        # if (response.ret.success == True) or (response.ret.code == 0):
        #    rospy.loginfo(response.ret.message)
        if (response.ret.success is False) or (response.ret.code == -1):
            self.logger.logerror(response.ret.message, "")

        return response

    def smtp_connection(self):
        """
        Establishes a connection to the SMTP server.

        Returns:
            bool: True if the connection is successfully established, False otherwise.
        """
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

        except smtplib.SMTPException as e:
            self.logger.logerror(
                f"smtp_manager::smtp_connection -> Exception: {e}", "")
            success = False

        return success

    def build_email(self, email_data):
        """
        Builds and returns an email message based on the provided email_data.

        Args:
            email_data (EmailData): The data object containing information for building the email.

        Returns:
            email (MIMEMultipart): The constructed email message.

        Raises:
            None
        """
        email = MIMEMultipart("alternative")

        # Set the Sender
        email["From"] = self.sender

        # Set the Subject
        if email_data.status.description == "":
            email["Subject"] = "default"
        else:
            email["Subject"] = email_data.status.description

        # Set the Attachments
        attachments, non_attachments = self.get_files_to_upload_as_attachments(
            email_data.files_to_upload)
        for attachment in attachments:
            email.attach(attachment)

        # Set the Massage
        if email_data.status.message == "":
            self.logger.logewarning("Message email is empty", "")
        non_attachments_msg = ''
        if non_attachments:
            non_attachments_msg = '<p>The following files could not be sent, as the maximum mail size was exceeded:</p>'
            for non_attachment in non_attachments:
                non_attachments_msg += f'<p>  - {non_attachment}</p>'

        email.attach(MIMEText(email_data.status.message +
                     non_attachments_msg, "html"))

        # Set the Recipient
        if '' in email_data.recipients or len(email_data.recipients) == 0:
            email["To"] = ', '.join(self.default_recipients)

        else:
            if self.check_recipients(email_data.recipients):
                email["To"] = ', '.join(email_data.recipients)
            else:
                email = None

        return email

    def send_email(self, email):
        """
        Sends an email using the SMTP server.

        Args:
            email (EmailMessage): The email message to be sent.

        Returns:
            bool: True if the email was sent successfully, False otherwise.
        """

        try:
            self.smtp.sendmail(
                email["From"], email["To"].split(','), email.as_string())
            success = True

        except smtplib.SMTPException as e:
            self.logger.logerror(
                f"smtp_manager::send_email -> Exception: {e}", "")
            success = False

        return success

    def get_files_to_upload_as_attachments(self, files_to_upload: List[str]) -> List[Union[str, MIMEApplication]]:
        """from 23 GB it is not allowed to send the email"""
        attachments, non_attachments = list(), list()
        total_size = 0.
        for file_to_upload in files_to_upload:
            with open(file_to_upload, 'rb') as f:
                attachment_data = f.read()

            attachment_size = round(len(attachment_data) / 1e6, 3)
            total_size += attachment_size
            if total_size < 22.5:
                attachment = MIMEApplication(
                    attachment_data, Name=basename(file_to_upload))
                attachment[
                    'Content-Disposition'] = f'attachment; filename="{basename(file_to_upload)}"'
                attachments.append(attachment)
            else:
                non_attachments.append(file_to_upload)

        if non_attachments:
            rospy.logwarn(
                f'Mail maximum size exceeded. Files {non_attachments} could not be sent as attachments')

        return attachments, non_attachments

    def smtp_disconnection(self):
        """
        Disconnects from the SMTP server.

        Returns:
            bool: True if the disconnection was successful, False otherwise.
        """
        try:
            self.smtp.quit()
        except smtplib.SMTPException as e:
            rospy.logerr(e)
            return False
        return True
