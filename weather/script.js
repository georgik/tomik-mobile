

document.getElementById('cloud').addEventListener('click', function() {
    document.getElementById('firstScreen').style.display = 'none'; // Hide main screen
    document.getElementById('secondScreen').style.display = 'flex'; // Show second screen

    // Example of setting dynamic content
    document.getElementById('temperature').innerText = 'Temperature: 24°C';
    document.getElementById('currentTime').innerText = `Current Time: ${new Date().toLocaleTimeString()}`;
});

document.addEventListener('DOMContentLoaded', function() {
    function updateTime() {
        document.getElementById('currentTime').innerText = `Current Time: ${new Date().toLocaleTimeString()}`;
    }

    updateTime(); // Update time immediately on load
    setInterval(updateTime, 1000); // Update time every second
});


document.getElementById('small-cloud').addEventListener('click', function() {
    document.getElementById('firstScreen').style.display = 'flex'; // Hide main screen
    document.getElementById('secondScreen').style.display = 'none'; // Show second screen
});


document.getElementById('temperature').addEventListener('click', function() {
    document.getElementById('tempDialog').style.display = 'flex'; // Show the dialog
});

function updateTemperature() {
    var newTemp = document.getElementById('tempInput').value; // Get the new temperature from input
    document.getElementById('temperature').innerText = `Temperature: ${newTemp}°C`; // Update the display
    document.getElementById('tempDialog').style.display = 'none'; // Hide the dialog
}
