#!/usr/bin/env python3
"""
Deploy ROS 2 Humble Desktop to Hugging Face Spaces
"""

from huggingface_hub import HfApi, create_repo, upload_folder
import os
import sys

def deploy_to_huggingface():
    """Deploy the current directory to Hugging Face Spaces"""
    
    # Get user credentials
    token = input("Enter your Hugging Face token (or press Enter if already logged in): ").strip()
    
    if token:
        api = HfApi(token=token)
    else:
        api = HfApi()
    
    # Get username for proper repo_id format
    try:
        user_info = api.whoami(token=token if token else None)
        username = user_info['name']
        print(f"✓ Authenticated as: {username}")
    except Exception as e:
        print(f"❌ Authentication failed: {e}")
        print("Please run 'huggingface-cli login' first or provide a valid token")
        return False
    
    # Repository details - must include username
    space_name = "huggingROS"
    repo_id = f"{username}/{space_name}"
    repo_type = "space"
    
    try:        # Try to create the repository (it may already exist)
        print(f"Creating/updating Space: {repo_id}")
        create_repo(
            repo_id=repo_id,
            repo_type=repo_type,
            space_sdk="docker",  # Changed to docker since we're using Dockerfile
            exist_ok=True,
            token=token if token else None,
            private=False  # Make it public
        )
        print("✓ Repository created/verified")
        
        # Files to ignore during upload
        ignore_patterns = [
            ".git/",
            ".devcontainer/",
            "DRL-robot-navigation/",
            "deploy_to_hf.py",
            "__pycache__/",
            "*.pyc",
            ".DS_Store",
            "HUGGING_FACE_DEPLOYMENT_GUIDE.md"
        ]
        
        # Upload the files
        print("Uploading files to Hugging Face Spaces...")
        upload_folder(
            folder_path=".",
            repo_id=repo_id,
            repo_type=repo_type,
            ignore_patterns=ignore_patterns,
            token=token if token else None
        )
        
        print("✅ Deployment successful!")
        print(f"🚀 Your Space is available at: https://huggingface.co/spaces/{repo_id}")
        print("⏰ Note: It may take a few minutes for the Space to build and become available.")
        
    except Exception as e:
        print(f"❌ Error during deployment: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure you're logged in: huggingface-cli login")
        print("2. Check your internet connection")
        print("3. Verify your Hugging Face token has write permissions")
        return False
    
    return True

if __name__ == "__main__":
    print("🤖 ROS 2 Humble Desktop - Hugging Face Spaces Deployment")
    print("=" * 60)
    
    # Check if we're in the right directory
    required_files = ["app.py", "requirements.txt", "Dockerfile", "README.md"]
    missing_files = [f for f in required_files if not os.path.exists(f)]
    
    if missing_files:
        print(f"❌ Missing required files: {missing_files}")
        print("Make sure you're in the correct directory with all deployment files.")
        sys.exit(1)
    
    print("✓ All required files found")
    print("Starting deployment...")
    
    success = deploy_to_huggingface()
    
    if success:
        print("\n🎉 Deployment completed successfully!")
        print("\nNext steps:")
        print("1. Visit your Space URL to see it building")
        print("2. Once built, test the desktop environment")
        print("3. Try connecting VS Code to the Space")
    else:
        print("\n❌ Deployment failed. Please check the error messages above.")
