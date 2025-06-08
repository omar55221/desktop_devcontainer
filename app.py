import gradio as gr
import subprocess
import os
import time
import threading

def start_desktop():
    """Start the ROS 2 desktop environment"""
    try:
        # Make sure script is executable
        subprocess.run(['chmod', '+x', './start_desktop.sh'], capture_output=True)
        result = subprocess.run(['./start_desktop.sh'], 
                              capture_output=True, text=True, timeout=60)
        return f"Desktop started!\n\nOutput:\n{result.stdout}\n{result.stderr}"
    except Exception as e:
        return f"Error starting desktop: {str(e)}"

def check_services():
    """Check if VNC and web services are running"""
    try:
        vnc_check = subprocess.run(['pgrep', '-f', 'Xtigervnc'], capture_output=True)
        web_check = subprocess.run(['pgrep', '-f', 'websockify'], capture_output=True)
        
        status = []
        status.append(f"VNC Server: {'✅ Running' if vnc_check.returncode == 0 else '❌ Stopped'}")
        status.append(f"Web Interface: {'✅ Running' if web_check.returncode == 0 else '❌ Stopped'}")
        status.append(f"Desktop Access: Navigate to the 6080 port tab above")
        
        return "\n".join(status)
    except Exception as e:
        return f"Error checking services: {str(e)}"

def auto_start_desktop():
    """Automatically start desktop services on app launch"""
    try:
        time.sleep(2)  # Wait a bit for the app to initialize
        subprocess.run(['chmod', '+x', './start_desktop.sh'], capture_output=True)
        subprocess.run(['./start_desktop.sh'], capture_output=True, timeout=60)
    except Exception as e:
        print(f"Auto-start failed: {e}")

# Auto-start desktop in background
threading.Thread(target=auto_start_desktop, daemon=True).start()

# Create Gradio interface
with gr.Blocks(title="ROS 2 Desktop Environment") as demo:
    gr.Markdown("# 🤖 ROS 2 Humble Desktop Environment")
    gr.Markdown("Start and manage your ROS 2 desktop environment in Hugging Face Spaces")
    
    with gr.Row():
        start_btn = gr.Button("🚀 Start Desktop Environment", variant="primary")
        check_btn = gr.Button("🔍 Check Services Status")
    
    output = gr.Textbox(label="Output", lines=10)
    
    start_btn.click(start_desktop, outputs=output)
    check_btn.click(check_services, outputs=output)
      gr.Markdown("""
    ## 🎯 Access Methods:
    - **Desktop**: Click the "6080" port tab above to access the noVNC desktop
    - **VNC Direct**: Port 5901 for VNC clients
    - **Terminal**: Right-click on the desktop to open terminal
    
    ## 🛠️ Available ROS 2 Tools:
    - **RViz2**: `rviz2` - 3D visualization tool
    - **Gazebo**: `gazebo` - Physics simulation
    - **RQT**: `rqt` - GUI tools for ROS
    - **Navigation2**: Pre-installed navigation stack
    - **TurtleBot3**: Example robot packages
    
    ## 🚀 Getting Started:
    1. Click "🚀 Start Desktop Environment" button above
    2. Wait for services to start
    3. Click the "6080" port tab to access desktop
    4. Open terminal and run: `rviz2` or `gazebo`
    """)

if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=7860, share=False)
