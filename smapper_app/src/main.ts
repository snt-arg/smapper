import axios from 'axios';

// Set up HTML structure with a spinner
document.querySelector<HTMLDivElement>('#app')!.innerHTML = `
  <div style="text-align: center; padding: 20px;">
    <h1>Scan Control</h1>
    <button id="start-sensors-button" style="margin-right: 10px; padding: 10px 20px;">Start Sensors</button>
    <button id="stop-sensors-button" style="padding: 10px 20px;">Stop Sensors</button>
    <button id="start-sgraphs-button" style="margin-right: 10px; padding: 10px 20px;">Start SGraphs</button>
    <button id="stop-sgraphs-button" style="padding: 10px 20px;">Stop SGraphs</button>
    <button id="save-scan-button" style="padding: 10px 20px;">Save Scan</button>
    <div id="spinner" style="display: none; margin-top: 20px;">
      <div class="spinner"></div>
      <p>Loading...</p>
    </div>
  </div>
`;

// CSS for spinner
const style = document.createElement("style");
style.innerHTML = `
  .spinner {
    width: 40px;
    height: 40px;
    border: 4px solid #ccc;
    border-top: 4px solid #333;
    border-radius: 50%;
    animation: spin 1s linear infinite;
    margin: 0 auto;
  }

  @keyframes spin {
    0% { transform: rotate(0deg); }
    100% { transform: rotate(360deg); }
  }
`;
document.head.appendChild(style);

// Function to control spinner visibility
const toggleSpinner = (show: boolean) => {
  const spinner = document.getElementById('spinner')!;
  spinner.style.display = show ? 'block' : 'none';
};

// Function to make POST requests with spinner control
const sendPostRequest = async (endpoint: string) => {
  toggleSpinner(true);
  try {
    const response = await axios.post(`http://localhost:8000/api${endpoint}`);
    console.log(`${endpoint} scan request successful:`, response.data);
  } catch (error) {
    console.error(`Error in ${endpoint} scan request:`, error);
  } finally {
    toggleSpinner(false); // Hide spinner after request completes
  }
};

// Attach event listeners to buttons
document.getElementById('start-sensors-button')!.addEventListener('click', () => {
  sendPostRequest('/sensors/lidar/start');
  sendPostRequest('/sensors/cameras/start');
});

document.getElementById('stop-sensors-button')!.addEventListener('click', () => {
  sendPostRequest('/sensors/lidar/stop');
  sendPostRequest('/sensors/cameras/stop');
});

// Attach event listeners to buttons
document.getElementById('start-sgraphs-button')!.addEventListener('click', () => {
  sendPostRequest('/sgraphs/start');
});

document.getElementById('stop-sgraphs-button')!.addEventListener('click', () => {
  sendPostRequest('/sgraphs/stop');
});

document.getElementById('save-scan-button')!.addEventListener('click', () => {
  sendPostRequest('/sgraphs/save');
});
