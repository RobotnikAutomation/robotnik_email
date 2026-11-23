# https://www.youtube.com/watch?v=ql5Dex4m40w
# https://www.gmass.co/smtp-test

from os.path import basename
from typing import List, Union

import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.application import MIMEApplication
import re
from datetime import datetime
import uuid
import time
from email import message_from_string
from .email_outbox import EmailOutbox
from robotnik_msgs.msg import State
from std_msgs.msg import UInt32


import rospy

from rcomponent.rcomponent import RComponent
from robotnik_alarms_msgs.srv import SendAlarms, SendAlarmsResponse, SendAlarmsRequest

smtp_response_codes = {
    211: "System status, or system help reply",
    214: "Help message",
    220: "Service ready",
    221: "Service closing transmission channel",
    250: "Requested mail action okay, completed",
    251: "User not local; will forward to <forward-path>",
    252: "Cannot VRFY user, but will accept message and attempt delivery",
    354: "Start mail input; end with <CRLF>.<CRLF>",
    421: "Service not available, closing transmission channel",
    450: "Requested mail action not taken: mailbox unavailable",
    451: "Requested action aborted: local error in processing",
    452: "Requested action not taken: insufficient system storage",
    500: "Syntax error, command unrecognized",
    501: "Syntax error in parameters or arguments",
    502: "Command not implemented",
    503: "Bad sequence of commands",
    504: "Command parameter not implemented",
    550: "Requested action not taken: mailbox unavailable",
    551: "User not local; please try <forward-path>",
    552: "Requested mail action aborted: exceeded storage allocation",
    553: "Requested action not taken: mailbox name not allowed",
    554: "Transaction failed"
}


class SMTPManager(RComponent):
    """
    SMTP server to send messages through ROS
    """
    SUCCESS = 0
    CONNECTION_FAILED = -1
    MALFORMED_EMAIL = -2
    QUEUE_OVERFLOW = -3
    INVALID_CONFIGURATION = -4
    NOT_READY = -5

    def __init__(self):

        RComponent.__init__(self)
        self.smtp_server = ''
        self.smtp_port = 0
        self.sender = ''
        self.use_authentication = False
        self.username = ''
        self.password = ''
        self.default_recipients = ''
        self.smtp = None
        self.send_email_service = None
        self.timeout = 10
        self.ssl = False
        self.tls = False
        self.ehlo = False
        self.max_mail_size = 20  # Maximum mail size in MB
        self.logger_tag = 'SMTP'
        # enables the auto generation of metadata like uuid and datetime
        self.auto_generate_uuid_datetime = True
        # enables the inclusion of all the detailed in the messages body
        self.include_detailed_info = True
        self.queue_max_len = 1000
        self.outbox = None
        self.last_cleanup = 0
        self.last_queue_log = 0
        self.time_between_emails = rospy.Duration(10)  # Maximum frequency of email sending
        self.email_last_sent_time = rospy.Time(0)
        self.ros_read_params()

        self.logger = self.initialize_logger()

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.smtp_server = rospy.get_param('~server', 'smtp.gmail.com')
        self.smtp_port = rospy.get_param('~port', 587)
        self.sender = rospy.get_param('~sender', 'default_sender@example.com')
        self.use_authentication = rospy.get_param(
            '~use_authentication', False)
        self.username = rospy.get_param('~username', 'username')
        self.password = rospy.get_param('~password', 'password')
        self.default_recipients = rospy.get_param(
            '~default_recipients', [])
        time_between_emails = rospy.get_param(
            '~time_between_emails', 5)
        self.ssl = rospy.get_param('~ssl', False)
        self.tls = rospy.get_param('~tls', False)
        self.ehlo = rospy.get_param('~ehlo', False)
        self.max_mail_size = rospy.get_param('~max_mail_size', 20)
        self.auto_generate_uuid_datetime = rospy.get_param(
            '~auto_generate_uuid_datetime', True)
        self.include_detailed_info = rospy.get_param(
            '~include_detailed_info', True)
        self.timeout = rospy.get_param('~timeout', 30)
        if isinstance(time_between_emails, (int, float)):
            self.time_between_emails = rospy.Duration(time_between_emails)
        self.queue_max_len = rospy.get_param('~queue_max_len', 1000)
        self.outbox_path = rospy.get_param('~outbox_path', '~/.ros/robotnik_email/outbox.sqlite3')
        self.batch_size = rospy.get_param('~batch_size', 10)
        self.batch_interval = rospy.get_param('~batch_interval', 60.0)
        self.retry_interval = rospy.get_param('~retry_interval', 60.0)

    def validate_configuration(self):
        """Validates the configuration of the SMTP manager"""
         # Validate default recipients
        len_default_recipients = len(self.default_recipients)
        self.default_recipients = self.get_valid_recipients(self.default_recipients)
        new_len_default_recipients = len(self.default_recipients)
        if len_default_recipients != new_len_default_recipients:
            self.logger.logwarning(
                "Some default recipients were invalid and were removed", self.logger_tag)

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        self.validate_configuration()
        if (not isinstance(self.queue_max_len, int) or self.queue_max_len < 1 or
                not isinstance(self.batch_size, int) or self.batch_size < 1 or
                any(not isinstance(v, (int, float)) or v < 0 for v in
                    (self.batch_interval, self.retry_interval, self.time_between_emails.to_sec()))):
            raise ValueError("Invalid outbox configuration")
        self.outbox = EmailOutbox(self.outbox_path, self.queue_max_len, self.batch_size,
                                  self.batch_interval, self.time_between_emails.to_sec(), self.retry_interval)
        
        # Service
        self.send_email_service = rospy.Service(
            '~send_email', SendAlarms, self.send_email_cb)
        self.queue_size_pub = rospy.Publisher('~queue_size', UInt32, queue_size=10)

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
            msg = f"default_recipients is malformed: {self.default_recipients}"
            rospy.logerr(msg)
            self.switch_to_state(State.FAILURE_STATE)
            return -1

        if self.check_recipients([self.sender]) is False:            
            msg = f"Sender is malformed: {self.sender}"
            rospy.logerr(msg)
            self.switch_to_state(State.FAILURE_STATE)
            return -1

        # SMTP availability is not a prerequisite for accepting or retrying
        # messages. The outbox worker will keep trying when configuration or
        # connectivity is repaired.
        self.switch_to_state(State.READY_STATE)
        return 0

    def ready_state(self):
        """Actions performed in ready state"""
        now = time.time()
        if now - self.last_queue_log >= 60:
            pending = self.outbox.count()
            rospy.loginfo("Email outbox: %d message(s) waiting to be sent", pending)
            self.last_queue_log = now
        if now - self.last_cleanup >= 3600:
            self.outbox.cleanup(now)
            self.last_cleanup = now
        item = self.outbox.next(now)
        if item:
            item_id, message, fallback = item
            try:
                success, detail, code = self.send_stored_email(message, fallback)
            except Exception as exc:
                success, detail = False, str(exc)
            self.outbox.record(item_id, success, detail if not success else '')
            if not success:
                rospy.logerr("Email %s failed and will be retried: %s", item_id, detail)

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
        if self.outbox:
            self.outbox.close()
            self.outbox = None
        return RComponent.shutdown(self)

    def all_state(self):
        """
        Publishes the current size of the email request queue and returns the state from the parent class.

        This method publishes the length of `self.email_request_queue` to the `queue_size_pub` publisher
        as a `UInt32` message. After publishing, it calls and returns the result of the `all_state` method
        from the `RComponent` superclass.

        Returns:
            The result of `RComponent.all_state(self)`.
        """
        # Publish the current size of the email request queue
        self.queue_size_pub.publish(UInt32(data=self.outbox.count() if self.outbox else 0))
        return RComponent.all_state(self)
        

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

        regex = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
        valid = True

        for recipient in recipients:
            if not re.search(regex, recipient):
                rospy.logerr(f"{recipient} is an invalid email")
                valid = False

        return valid
    
    def get_valid_recipients(self, recipients):
        """
        Checks if the given recipients are valid email addresses.

        Args:
            recipients (list): A list of email addresses to be checked.

        Returns:
            list: Valid email recipients.
        """

        valid_recipients = []

        for recipient in recipients:
            if not self.is_valid_email(recipient):
                rospy.logerr(f"{recipient} is an invalid email")
            else:
                valid_recipients.append(recipient)

        return valid_recipients

    def is_valid_email(self, email):
        """
        Validates the format of an email address.

        Args:
            email (str): The email address to be validated.

        Returns:
            bool: True if the email address is valid, False otherwise.
        """
        regex = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
        return re.search(regex, email) is not None

    def send_email_cb(self, req: SendAlarmsRequest):
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
        response.ret.message = ""

        if self._state != State.READY_STATE:
            msg = f"The node is not in READY_STATE, current state: {self.state_to_string(self._state)}"
            rospy.logerr(msg)
            response.ret.code = self.NOT_READY
            response.ret.message = msg
            return response
        try:
            email = self.build_email(req)
            fallback = self.build_email(req, send_with_attachments=False)
            if email is None or fallback is None or not email.get("To"):
                response.ret.code = self.MALFORMED_EMAIL
                response.ret.message = "Invalid or empty recipients"
                return response
            item_id = self.outbox.enqueue(email.as_string(), fallback.as_string())
            if item_id is None:
                response.ret.code = self.QUEUE_OVERFLOW
                response.ret.message = "Email queue is full"
                return response
        except Exception as exc:
            rospy.logerr("Cannot persist email: %s", exc)
            response.ret.message = "Cannot persist email: %s" % exc
            return response

        response.ret.code = self.SUCCESS
        response.ret.success = True
        response.ret.message = "Email request received and will be processed"

        return response
    

    def send_stored_email(self, message, fallback):
        connected, detail, code = self.smtp_connection()
        if not connected:
            return False, detail, code
        try:
            result = self.send_email(message_from_string(message))
            if not result[0] and result[2] == 552:
                result = self.send_email(message_from_string(fallback))
            return result
        finally:
            self.smtp_disconnection()

    def process_and_send(self, req):
        """
        Sends an email using the configured SMTP server, with optional attachments.
        Attempts to establish a connection to the SMTP server and send an email constructed from the provided request.
        If sending with attachments fails due to storage limitations (error code 552), it retries once without attachments.
        Handles malformed emails and connection errors, and logs relevant information and errors.
        Args:
            req: The request object containing email details (such as recipients, subject, body, and attachments).
        Returns:
            success (bool), ret_msg (str), ret_code (int): A tuple containing the success status, response code, and message.
            
            SUCCESS: If the email is sent successfully.
            CONNECTION_FAILED: If the SMTP server connection fails, it cannot send the email.
            MALFORMED_EMAIL: If the email is malformed, it cannot be sent.
        """
        
        ret = False
        ret_connection = False
        ret_msg = ''
        ret_code = self.SUCCESS
        try_send = True
        send_with_attachments = True

        # try to connect to the SMTP server
        ret_connection, ret_msg, ret_code = self.smtp_connection()

        if ret_connection is True:

            while try_send and ret is False:
                # Check if the request is valid
                email = self.build_email(
                    req, send_with_attachments=send_with_attachments)

                if email is not None:
                    ret, ret_msg, ret_code = self.send_email(email)
                    if ret is True:
                        self.logger.loginfo(
                            "Email sent from " + email["From"] + " to " + email["To"], self.logger_tag)
                        return True, ret_msg, self.SUCCESS
                    else:
                        # 552: Requested mail action aborted: exceeded storage allocation
                        if ret_code == 552:
                            if send_with_attachments is False:
                                ret_msg = f"The email could not be sent: {ret_msg}"
                                try_send = False
                            else:
                                # Try once without any attachments
                                send_with_attachments = False
                                try_send = True
                        else:
                            ret_msg = f"The email could not be sent: {ret_msg}"
                            try_send = False

                else:
                    return False, "The email can not be sent because it is malformed", self.MALFORMED_EMAIL

            try:
                self.smtp_disconnection()
            except smtplib.SMTPException as e:
                rospy.logerr(e)

            rospy.logerr("Cannot send email: " + ret_msg)
            return False, ret_msg, ret_code
        else:

            msg = "Cannot connect to SMTP server " + \
                str(self.smtp_server) + " with port " + \
                str(self.smtp_port) + ": " + ret_msg
            rospy.logerr(msg)
            return False, msg, ret_code


    def smtp_connection(self):
        """
        Establishes a connection to the SMTP server.

        Returns:
            A tuple containing:
                - bool: True if the connection is successfully established, False otherwise.
                - str: A message indicating the result of the connection attempt.
                - int: A response code indicating the result of the connection attempt.
        """

        ret_msg = 'OK'

        # Validate SMTP server and port
        if not self.smtp_server or not isinstance(self.smtp_server, str):
            ret_msg = f"Invalid SMTP server configuration: {self.smtp_server}"
            rospy.logerr(ret_msg)
            return False, ret_msg, self.INVALID_CONFIGURATION

        if not self.smtp_port or not isinstance(self.smtp_port, int):
            ret_msg = f"Invalid SMTP port configuration: {self.smtp_port}"
            rospy.logerr(ret_msg)
            return False, ret_msg, self.INVALID_CONFIGURATION

        try:
            if self.ssl:
                rospy.loginfo(
                    f"Connecting to SMTP server {self.smtp_server} on port {self.smtp_port} with SSL. Timeout: {self.timeout}")
                self.smtp = smtplib.SMTP_SSL(
                    self.smtp_server, port=self.smtp_port, timeout=self.timeout)
            else:
                self.smtp = smtplib.SMTP(
                    self.smtp_server, port=self.smtp_port, timeout=self.timeout)
                rospy.loginfo(
                    f"Connecting to SMTP server {self.smtp_server} on port {self.smtp_port}. Timeout: {self.timeout}")

            self.smtp.connect(self.smtp_server, self.smtp_port)
            if self.ehlo:
                self.smtp.ehlo()
            if self.tls:
                self.smtp.starttls()
                if self.ehlo:  # Requires EHLO after STARTTLS
                    self.smtp.ehlo()

            if self.use_authentication:
                self.smtp.login(self.username, self.password)

            success = True

        except smtplib.SMTPException as e:
            rospy.logerr_throttle(10.0, f"SMTPException: {e}")
            ret_msg = f"{e}"
            success = False
        except Exception as e:
            rospy.logerr_throttle(10.0, f"smtp_connection -> Exception: {e}")
            ret_msg = f"{e}"
            success = False

        return success, ret_msg, self.SUCCESS if success else self.CONNECTION_FAILED

    def build_email(self, email_data, send_with_attachments=True):
        """
        Builds and returns an email message based on the provided email_data.

        Args:
            email_data (EmailData): The data object containing information for building the email.
            send_with_attachments (bool): Flag indicating whether to include attachments in the email. Default is True.

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

        non_attachments_msg = ''
        # Set the Attachments
        rospy.loginfo(f'Files to upload: {email_data.files_to_upload}')

        if email_data.files_to_upload is not None and len(email_data.files_to_upload) > 0:

            if send_with_attachments is True:
                attachments, non_attachments = self.get_files_to_upload_as_attachments(
                    email_data.files_to_upload)
                for attachment in attachments:
                    email.attach(attachment)

                if non_attachments:
                    non_attachments_msg = '<p>Errors: The following files could not be sent, as the maximum mail size was exceeded:</p>'
                    for non_attachment in non_attachments:
                        non_attachments_msg += f'<p>  - {non_attachment}</p>'
            # Send with no attachments, but list them
            else:
                non_attachments_msg = '<p>Errors: The following files could not be attached:</p>'
                for non_attachment in email_data.files_to_upload:
                    non_attachments_msg += f'<p>  - {non_attachment}</p>'
        # Set the Message
        if email_data.status.message == "":
            rospy.logwarn("Message email is empty")

        datetime_msg = 'Date: '
        uuid_msg = 'Uuid: '
        if self.auto_generate_uuid_datetime:
            datetime_msg += email_data.datetime if email_data.datetime != '' else datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            uuid_msg += email_data.uuid if email_data.uuid != '' else str(
                uuid.uuid4())
        else:
            datetime_msg += email_data.datetime
            uuid_msg += email_data.uuid

        id_msg = email_data.status.id
        type_msg = email_data.status.type

        msg = ''
        if self.include_detailed_info:
            msg = f'<p>{datetime_msg}</p><p>{uuid_msg}</p><p>ID: {id_msg}</p><p>Type: {type_msg}</p>'
        msg += f'<p>Message: {email_data.status.message}</p>'
        msg += non_attachments_msg

        email.attach(MIMEText(msg, "html"))

        # Set the Recipient
        if isinstance(email_data.recipients, list) and ('' in email_data.recipients or len(email_data.recipients) == 0):
            email["To"] = ', '.join(self.default_recipients)

        else:
            valid_recipients = self.get_valid_recipients(email_data.recipients)
            if valid_recipients:
                email["To"] = ', '.join(valid_recipients)
            else:
                email = None

        return email

    def send_email(self, email):
        """
        Sends an email using the SMTP server.

        Args:
            email (EmailMessage): The email message to be sent.

        Returns:
            Tuple[bool, str, int]: A tuple containing the following:
                - success (bool): True if the email was sent successfully, False otherwise.
                - ret_msg (str): A message indicating the result of the email sending process.
                - ret_code (int): A code indicating the result of the email sending process.
        """
        rospy.loginfo(f"Sending email from {email['From']} to {email['To']}")
        ret_msg = ''
        ret_code = self.SUCCESS
        success = False

        try:
            recipients = email["To"].split(',')
            refused = self.smtp.sendmail(email["From"], [r.strip() for r in recipients], email.as_string())
            if refused:
                return False, "Recipients refused: %s" % refused, self.CONNECTION_FAILED
            ret_msg = f"Email sent from {email['From']} to {email['To']}"
            success = True

        except AttributeError as e:
            
            ret_code = self.MALFORMED_EMAIL
            rospy.logerr_throttle(30.0, f"Invalid 'To' field format. Expected a comma-separated string: {e}")
            ret_msg = f"Invalid 'To' field format. Expected a comma-separated string: {e}"
            success = False
            rospy.logerr(ret_msg)
            #raise ValueError("Invalid format for 'To' field. Expected a comma-separated string.")
        except smtplib.SMTPResponseException as e:

            ret_code = e.smtp_code
            rospy.logerr_throttle(30.0, f"SMTPResponseException: {e.smtp_code} {e.smtp_error}")
            ret_msg = f"{e.smtp_code} {e.smtp_error}"
            success = False
        except smtplib.SMTPServerDisconnected as e:

            rospy.logerr_throttle(30.0, f"smtp_manager::send_email -> Exception: {e}")
            ret_msg = f"{e}"
            success = False
            ret_code = self.CONNECTION_FAILED
        except smtplib.SMTPException as e:

            rospy.logerr_throttle(30.0, f'errno = {type(e)}')
            rospy.logerr_throttle(30.0, f"smtp_manager::send_email -> Exception: {e}")
            ret_msg = f"{e}"
            success = False
            ret_code = self.CONNECTION_FAILED

        return success, ret_msg, ret_code

    def get_files_to_upload_as_attachments(self, files_to_upload: List[str]) -> List[Union[str, MIMEApplication]]:
        """
        Retrieves a list of files to be uploaded as attachments.

        Args:
            files_to_upload (List[str]): A list of file paths to be uploaded as attachments.

        Returns:
            attachments (List[Union[str, MIMEApplication]]): A list of attachments to be included in the email.
            non_attachments (List[str]): A list of files that could not be attached due to exceeding the maximum mail size.
        """
        attachments, non_attachments = list(), list()
        total_size = 0.
        for file_to_upload in files_to_upload:

            if file_to_upload != '':
                try:
                    rospy.loginfo(f'Uploading file: {file_to_upload}')
                    with open(file_to_upload, 'rb') as f:
                        attachment_data = f.read()

                    attachment_size = round(len(attachment_data) / (1024 * 1024), 3)
                    total_size += attachment_size
                    if total_size < self.max_mail_size:
                        attachment = MIMEApplication(
                            attachment_data, Name=basename(file_to_upload))
                        attachment[
                            'Content-Disposition'] = f'attachment; filename="{basename(file_to_upload)}"'
                        attachments.append(attachment)
                    else:
                        msg = f'File {file_to_upload} ({attachment_size} MB) could not be attached due to the total size ({total_size} MB) is exceeding the maximum mail size of {self.max_mail_size} MB'
                        rospy.logwarn(msg)
                        self.logger.logwarning(msg, self.logger_tag)
                        non_attachments.append(file_to_upload)
                except FileNotFoundError:
                    msg = f'File {file_to_upload} could not be found. It will not be sent as an attachment'
                    rospy.logerr(msg)
                    rospy.logerr(f'File {file_to_upload} could not be found. It will not be sent as an attachment')

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
        if self.smtp:
            try:
                self.smtp.quit()
                self.smtp = None  # Reset the SMTP connection to None after quitting
            except smtplib.SMTPException as e:
                rospy.logerr(e)
                return False
        else:
            rospy.logwarn("SMTP connection is already None. No disconnection needed.")
        return True

    def ret_code_to_string(self, code):
        """
        Converts a return code into a human-readable string.

        Args:
            code (int): The return code.

        Returns:
            str: The string representation of the return code.
        """
        code_map = {
            self.SUCCESS: "Success",
            self.CONNECTION_FAILED: "Connection failed",
            self.MALFORMED_EMAIL: "Malformed email",
            self.QUEUE_OVERFLOW: "Queue overflow",
            self.INVALID_CONFIGURATION: "Invalid configuration",
            self.NOT_READY: "Not ready",
        }
        return code_map.get(code, f"Unknown code: {code}")