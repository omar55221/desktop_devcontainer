#!/usr/bin/env python3
"""
Test script to verify the ROS 2 Desktop Environment deployment
"""
import requests
import time

def test_hf_space():
    """Test if the Hugging Face Space is working"""
    space_url = "https://huggingface.co/spaces/om2030/huggingROS"
    
    print("🔍 Testing Hugging Face Space deployment...")
    print(f"Space URL: {space_url}")
    
    try:
        response = requests.get(space_url, timeout=10)
        print(f"✅ Space accessibility: HTTP {response.status_code}")
        
        if response.status_code == 200:
            content = response.text.lower()
            
            # Check for key indicators
            indicators = [
                ("gradio", "Gradio interface"),
                ("ros", "ROS 2 content"),
                ("desktop", "Desktop environment"),
                ("vnc", "VNC setup")
            ]
            
            print("\n📊 Content Analysis:")
            for keyword, description in indicators:
                if keyword in content:
                    print(f"✅ {description}: Found")
                else:
                    print(f"❌ {description}: Not found")
                    
            # Check for error indicators
            error_indicators = ["error", "failed", "exception", "traceback"]
            errors_found = [err for err in error_indicators if err in content]
            
            if errors_found:
                print(f"\n⚠️ Potential errors detected: {', '.join(errors_found)}")
            else:
                print("\n✅ No obvious errors detected in content")
                
    except requests.RequestException as e:
        print(f"❌ Error accessing space: {e}")
        
    print(f"\n🌐 Open the space in your browser: {space_url}")

if __name__ == "__main__":
    test_hf_space()
