#ifndef OTA_UPLOAD_PAGE_H
#define OTA_UPLOAD_PAGE_H

const char* OTA_UPLOAD_HTML = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>RFx4 OTA Firmware Update</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 800px;
            margin: 50px auto;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
        }
        .container {
            background: white;
            border-radius: 10px;
            padding: 30px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
        }
        h1 {
            color: #333;
            text-align: center;
            margin-bottom: 10px;
        }
        .subtitle {
            text-align: center;
            color: #666;
            margin-bottom: 30px;
        }
        .info-box {
            background: #e3f2fd;
            border-left: 4px solid #2196F3;
            padding: 15px;
            margin-bottom: 20px;
            border-radius: 4px;
        }
        .form-group {
            margin-bottom: 20px;
        }
        label {
            display: block;
            font-weight: bold;
            margin-bottom: 8px;
            color: #333;
        }
        input[type="file"] {
            padding: 10px;
            border: 2px solid #ddd;
            border-radius: 4px;
            width: 100%;
            box-sizing: border-box;
        }
        button {
            background: #667eea;
            color: white;
            padding: 12px 30px;
            border: none;
            border-radius: 4px;
            font-size: 16px;
            cursor: pointer;
            width: 100%;
            transition: background 0.3s;
        }
        button:hover {
            background: #764ba2;
        }
        button:disabled {
            background: #ccc;
            cursor: not-allowed;
        }
        .progress-container {
            display: none;
            margin-top: 20px;
        }
        #progressBar {
            width: 100%;
            height: 30px;
            background: #f0f0f0;
            border-radius: 4px;
            overflow: hidden;
        }
        #progressFill {
            height: 100%;
            background: linear-gradient(90deg, #667eea, #764ba2);
            width: 0%;
            transition: width 0.3s;
            display: flex;
            align-items: center;
            justify-content: center;
            color: white;
            font-weight: bold;
        }
        #status {
            margin-top: 10px;
            padding: 10px;
            border-radius: 4px;
            text-align: center;
            font-weight: bold;
        }
        .status-info { background: #e3f2fd; color: #1976d2; }
        .status-success { background: #e8f5e9; color: #388e3c; }
        .status-error { background: #ffebee; color: #d32f2f; }
        .system-info {
            background: #f5f5f5;
            padding: 15px;
            border-radius: 4px;
            margin-top: 20px;
        }
        .system-info p {
            margin: 8px 0;
            color: #555;
            font-family: monospace;
        }
        .warning {
            background: #fff3e0;
            border-left: 4px solid #ff9800;
            padding: 15px;
            margin-bottom: 20px;
            border-radius: 4px;
            color: #e65100;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>RFx4 OTA Update</h1>
        <p class="subtitle">Wirelessly update your device firmware</p>
        <div style="text-align: center; margin-bottom: 20px;">
            <a href="/" style="color: #667eea; text-decoration: none; font-size: 14px;">← Back to Dashboard</a>
        </div>
        
        <div class="warning">
            <strong>[WARNING] Important:</strong> Do not disconnect power or WiFi during the update process. The device will automatically reboot when complete.
        </div>
        
        <div class="info-box">
            <strong>[INFO] How to use:</strong>
            <ol style="margin: 10px 0; padding-left: 25px;">
                <li>Select a <code>.bin</code> file from your PlatformIO build</li>
                <li>Click "Upload Firmware" to start</li>
                <li>Wait for the progress bar to complete</li>
                <li>Device will reboot automatically with new firmware</li>
            </ol>
        </div>
        
        <form id="uploadForm">
            <div class="form-group">
                <label for="firmwareFile">Select Firmware File (.bin):</label>
                <input type="file" id="firmwareFile" name="firmware" accept=".bin" required>
            </div>
            <button type="button" onclick="uploadFirmware()">Upload Firmware</button>
        </form>
        
        <div class="progress-container" id="progressContainer">
            <div id="progressBar">
                <div id="progressFill">0%</div>
            </div>
            <div id="status" class="status-info">Initializing...</div>
        </div>
        
        <div class="system-info">
            <p><strong>System Information:</strong></p>
            <p>Current Partition: <span id="currentPartition">Loading...</span></p>
            <p>Available Space: <span id="availableSpace">Loading...</span></p>
            <p>Flash Size: <span id="flashSize">Loading...</span></p>
            <p>Version: <span id="version">Loading...</span></p>
        </div>

        <hr style="border: none; border-top: 1px solid #ddd; margin: 30px 0;">

        <h2 style="color: #444; text-align: center; margin-bottom: 10px;">Update Dashboard File</h2>
        <p class="subtitle">Upload a new file to SPIFFS &mdash; live immediately, no reboot needed</p>

        <div class="info-box">
            <strong>[INFO] How to use:</strong>
            <ol style="margin: 10px 0; padding-left: 25px;">
                <li>Select your updated <code>dashboard.html</code> (or any other SPIFFS file)</li>
                <li>Click "Upload File"</li>
                <li>Refresh <a href="/">/</a> to see the new version &mdash; no reboot required</li>
            </ol>
        </div>

        <form id="spiffsForm">
            <div class="form-group">
                <label for="spiffsFile">Select File to Upload:</label>
                <input type="file" id="spiffsFile" name="spiffs" accept=".html,.css,.js,.json,.txt" required>
            </div>
            <button type="button" onclick="uploadSPIFFS()">Upload File</button>
        </form>

        <div class="progress-container" id="spiffsProgressContainer">
            <div style="width:100%;height:30px;background:#f0f0f0;border-radius:4px;overflow:hidden;">
                <div id="spiffsProgressFill" style="height:100%;background:linear-gradient(90deg,#667eea,#764ba2);width:0%;transition:width 0.3s;display:flex;align-items:center;justify-content:center;color:white;font-weight:bold;">0%</div>
            </div>
            <div id="spiffsStatus" class="status-info">Ready</div>
        </div>
    </div>

    <script>
        // Load system info on page load
        window.onload = function() {
            loadSystemInfo();
        };

        function loadSystemInfo() {
            fetch('/ota/info')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('currentPartition').textContent = data.current_partition || 'Unknown';
                    document.getElementById('availableSpace').textContent = 
                        (data.available_space ? (data.available_space / 1024 / 1024).toFixed(2) : '0') + ' MB';
                    document.getElementById('flashSize').textContent = 
                        (data.flash_size ? (data.flash_size / 1024 / 1024).toFixed(2) : '0') + ' MB';
                    document.getElementById('version').textContent = data.version || 'Unknown';
                })
                .catch(err => {
                    console.error('Error loading system info:', err);
                    document.getElementById('version').textContent = 'Error loading info';
                });
        }

        function uploadFirmware() {
            const fileInput = document.getElementById('firmwareFile');
            const file = fileInput.files[0];
            
            if (!file) {
                alert('Please select a firmware file');
                return;
            }
            
            if (!file.name.endsWith('.bin')) {
                alert('Please select a .bin file');
                return;
            }
            
            const progressContainer = document.getElementById('progressContainer');
            const progressFill = document.getElementById('progressFill');
            const status = document.getElementById('status');
            const uploadBtn = document.querySelector('#uploadForm button');
            
            progressContainer.style.display = 'block';
            uploadBtn.disabled = true;
            fileInput.disabled = true;
            
            status.className = 'status-info';
            status.textContent = '[UPLOADING] 0%';
            progressFill.style.width = '0%';
            progressFill.textContent = '0%';
            
            const xhr = new XMLHttpRequest();
            
            xhr.upload.addEventListener('progress', (e) => {
                if (e.lengthComputable) {
                    const percentComplete = (e.loaded / e.total) * 100;
                    progressFill.style.width = percentComplete + '%';
                    progressFill.textContent = Math.round(percentComplete) + '%';
                    status.textContent = '[UPLOADING] ' + Math.round(percentComplete) + '%';
                    status.className = 'status-info';
                }
            });
            
            xhr.addEventListener('load', () => {
                if (xhr.status === 200) {
                    progressFill.style.width = '100%';
                    progressFill.textContent = '100%';
                    status.className = 'status-success';
                    status.textContent = '[OK] Upload successful! Device will reboot in a few seconds...';
                    setTimeout(() => {
                        alert('Firmware update complete! Device is rebooting...');
                        location.reload();
                    }, 3000);
                } else {
                    status.className = 'status-error';
                    status.textContent = '[ERROR] Update failed (HTTP ' + xhr.status + '). Check console for details.';
                }
                uploadBtn.disabled = false;
                fileInput.disabled = false;
            });
            
            xhr.addEventListener('error', () => {
                status.className = 'status-error';
                status.textContent = '[ERROR] Upload failed. Check your connection.';
                uploadBtn.disabled = false;
                fileInput.disabled = false;
            });
            
            const formData = new FormData();
            formData.append('firmware', file, file.name);
            xhr.open('POST', '/update', true);
            xhr.send(formData);
        }

        function uploadSPIFFS() {
            const fileInput = document.getElementById('spiffsFile');
            const file = fileInput.files[0];

            if (!file) {
                alert('Please select a file');
                return;
            }

            const progressContainer = document.getElementById('spiffsProgressContainer');
            const progressFill = document.getElementById('spiffsProgressFill');
            const status = document.getElementById('spiffsStatus');
            const uploadBtn = document.querySelector('#spiffsForm button');

            progressContainer.style.display = 'block';
            uploadBtn.disabled = true;
            fileInput.disabled = true;

            status.className = 'status-info';
            status.textContent = 'Uploading...';
            progressFill.style.width = '0%';
            progressFill.textContent = '0%';

            const xhr = new XMLHttpRequest();

            xhr.upload.addEventListener('progress', (e) => {
                if (e.lengthComputable) {
                    const pct = Math.round((e.loaded / e.total) * 100);
                    progressFill.style.width = pct + '%';
                    progressFill.textContent = pct + '%';
                    status.textContent = 'Uploading ' + pct + '%';
                }
            });

            xhr.addEventListener('load', () => {
                if (xhr.status === 200) {
                    progressFill.style.width = '100%';
                    progressFill.textContent = '100%';
                    status.className = 'status-success';
                    status.textContent = '\u2713 File uploaded! Refresh the dashboard to see changes.';
                } else {
                    status.className = 'status-error';
                    status.textContent = 'Upload failed (HTTP ' + xhr.status + ')';
                }
                uploadBtn.disabled = false;
                fileInput.disabled = false;
            });

            xhr.addEventListener('error', () => {
                status.className = 'status-error';
                status.textContent = 'Upload failed. Check your connection.';
                uploadBtn.disabled = false;
                fileInput.disabled = false;
            });

            const formData = new FormData();
            formData.append('spiffs', file, file.name);
            xhr.open('POST', '/update-spiffs', true);
            xhr.send(formData);
        }
    </script>
</body>
</html>
)rawliteral";

#endif // OTA_UPLOAD_PAGE_H
