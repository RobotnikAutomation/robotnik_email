#!/usr/bin/env python3
import unittest
import rostest

try:
    from robotnik_email import SMTPManager
except ImportError:
    from robotnik_email.smtp_manager import SMTPManager

class TestSMTPManager(unittest.TestCase):
    """Unit tests for SMTPManager."""

    @classmethod
    def setUpClass(cls):
        """Set up the SMTPManager instance for all tests."""
        cls.smtp_manager = SMTPManager()

    def test_is_valid_email(self):
        """Test the is_valid_email method with valid and invalid email addresses."""
        valid_emails = [
            "test@example.com",
            "user.name+tag+sorting@example.com",
            "user.name@example.co.uk",
            "user_name@example.com",
            "username@subdomain.example.com",
            "user-name@example.com",
            "user.name@domain.com",
            "user.name@domain.co.in",
            "user-name@domain.com",
            "user_name@domain.com",
            "username@domain.com",
            "user@domain.name",
            "user@domain.co.jp",
            "user@domain.web",
            "user@domain.org",
            "user@domain.co",
            "user@domain.info",
            "user@domain.biz",
            "test@unitedrobotics.group"
        ]

        for email in valid_emails:
            with self.subTest(email=email):
                ret = self.smtp_manager.is_valid_email(email)
                print(f'It is {email} valid? {ret}')
                self.assertTrue(
                    ret,
                    f"Expected {email} to be valid"
                )
    
    def test_is_invalid_email(self):
        """Test the is_valid_email method with invalid email addresses."""

        invalid_emails = [
            "plainaddress",
            "@missingusername.com",
            "username@.com",
            "username@.com.",
            "username@domain,com",
            "username@domain@domain.com",
            "username@domain.com (Joe Smith)",
            "username@domain.com.",
            "username@domain",
            "username@domain.c"
        ]

        for email in invalid_emails:
            with self.subTest(email=email):
                ret = self.smtp_manager.is_valid_email(email)
                print(f'It is {email} valid? {ret}')
                self.assertFalse(
                    ret,
                    f"Expected {email} to be invalid"
                )

if __name__ == '__main__':
    rostest.rosrun('smtp_manager', 'test_smtp_manager', TestSMTPManager)
