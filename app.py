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
        status.append(f"VNC Server: {'[RUNNING]' if vnc_check.returncode == 0 else '[STOPPED]'}")        
        status.append(f"Web Interface: {'[RUNNING]' if web_check.returncode == 0 else '[STOPPED]'}")     
        status.append(f"Desktop Access: Navigate to the 6080 port tab above")
        
        return "\n".join(status)
    except Exception as e:
        return f"Error checking services: {str(e)}"

def auto_start_desktop():
    """Auto-start desktop environment in background"""
    time.sleep(5)
    start_desktop()

# Auto-start desktop in background
threading.Thread(target=auto_start_desktop, daemon=True).start()

# Create Gradio interface
with gr.Blocks(title="ROS 2 Desktop Environment") as demo:
    gr.Markdown("# 🖥️ ROS 2 Humble Desktop Environment")
    gr.Markdown("**Complete ROS 2 desktop environment running in your browser!**")
    
    # Main control buttons
    with gr.Row():
        start_btn = gr.Button("🚀 START DESKTOP", variant="primary", size="lg")
        check_btn = gr.Button("📊 CHECK STATUS", variant="secondary")
    
    output = gr.Textbox(label="System Output", lines=8, interactive=False)
    
    # Desktop access section
    gr.Markdown("## 🖥️ Desktop Access")
    gr.Markdown("""
    **After clicking START DESKTOP above:**
    1. Wait for services to initialize (check output above)
    2. Look for port tabs at the top of this page
    3. Click the **"6080"** port tab to access your desktop
    4. If no port tab appears, the service may still be starting
    """)
    
    # Button actions
    start_btn.click(start_desktop, outputs=output)
    check_btn.click(check_services, outputs=output)    
    gr.Markdown("""
    ## 🛠️ Available Tools Once Desktop Loads:
    - **RViz2**: `rviz2` - 3D visualization
    - **Gazebo**: `gazebo` - Physics simulation  
    - **RQT**: `rqt` - GUI tools for ROS
    - **Terminal**: Right-click desktop → Open Terminal
    
    ## 💡 Troubleshooting:
    - **No desktop?** → Click START DESKTOP, wait 30-60 seconds, look for port 6080 tab
    - **Services not running?** → Click CHECK STATUS to verify
    - **Need terminal?** → Right-click on desktop after accessing port 6080
    """)

if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=7860, share=False)
