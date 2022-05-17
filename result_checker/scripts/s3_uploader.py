import boto3
from botocore.exceptions import NoCredentialsError

ACCESS_KEY = 'AKIAUNX7XU6MRV3R4KWV'
SECRET_KEY = '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'


def upload_to_aws(file_path, bucket, s3_file):
    # s3 = boto3.client('s3', aws_access_key_id=ACCESS_KEY,
    #                   aws_secret_access_key=SECRET_KEY)

    # s3 = boto3.client('s3')

    # s3.upload_file(file_path, bucket, s3_file, ExtraArgs={'ACL' : 'public-read'})
    s3 = boto3.resource('s3')
    s3.meta.client.upload_file(file_path,bucket,s3_file)
    print("Upload Successful")
    # try:
    #     s3.upload_file(file_path, bucket, s3_file)
    #     print("Upload Successful")
    #     return True
    # except :
    #     print("error")
    #     print(file_path)
    #     return False
