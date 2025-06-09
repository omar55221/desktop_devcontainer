import gradio as gr
import subprocess
import os
import time
import threading

def start_desktop():
    """Start the ROS 2 desktop environment"""
    try:
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
        status.append(f"VNC Server: {'RUNNING' if vnc_check.returncode == 0 else 'STOPPED'}")        
        status.append(f"Web Interface: {'RUNNING' if web_check.returncode == 0 else 'STOPPED'}")     
        status.append(f"Desktop Access: Navigate to the 6080 port tab above")
        
        return "\n".join(status)
    except Exception as e:
        return f"Error checking services: {str(e)}"

# Create simple Gradio interface
with gr.Blocks(title="ROS 2 Desktop Environment") as demo:
    gr.Markdown("# ROS 2 Humble Desktop Environment")
    gr.Markdown("Click START DESKTOP, then look for port 6080 tab above")
    
    with gr.Row():
        start_btn = gr.Button("START DESKTOP", variant="primary")
        check_btn = gr.Button("CHECK STATUS")
    
    output = gr.Textbox(label="System Output", lines=6)
    
    start_btn.click(start_desktop, outputs=output)
    check_btn.click(check_services, outputs=output)
    
    gr.Markdown("After clicking START DESKTOP, wait 30 seconds then look for the '6080' port tab at the top of this page.")

if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=7860, share=False)
