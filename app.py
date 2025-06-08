import gradio as gr
import subprocess
import os
import time

def start_desktop():
    """Start the ROS 2 desktop environment"""
    try:
        result = subprocess.run(['./start_desktop.sh'], 
                              capture_output=True, text=True, timeout=30)
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
        status.append(f"Desktop Access: http://localhost:6080")
        
        return "\n".join(status)
    except Exception as e:
        return f"Error checking services: {str(e)}"

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
    - **Desktop**: Click the "6080" port tab above or visit the noVNC URL
    - **VS Code**: Use the VS Code button if available
    - **SSH**: Connect via SSH if enabled
    
    ## 🛠️ Available Tools:
    - RViz2: `rviz2`
    - Gazebo: `gazebo`
    - RQT: `rqt`
    - Terminal: Right-click desktop
    """)

if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=7860)
