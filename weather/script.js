

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


document.addEventListener('DOMContentLoaded', function() {
    // Existing code...
    setupPhaser(); // Setup Phaser after the document is ready
});

function setupPhaser() {
    var config = {
        type: Phaser.AUTO,
        width: 64,  // Adjusted for the scaled size
        height: 64, // Adjusted for the scaled size
        parent: 'phaser-game', // This is the id of the div where Phaser will render
        transparent: true,
        physics: {
            default: 'arcade',
            arcade: {
                gravity: { y: 0 },
                debug: false
            }
        },
        scale: {
            mode: Phaser.Scale.NONE, // Prevent Phaser from resizing automatically
            parent: 'phaser-game',
            width: 64, // Original width
            height: 64, // Original height
            zoom: 2 // Zoom level for the game (2x size)
        },
        scene: {
            preload: preload,
            create: create
        }
    };

    var game = new Phaser.Game(config);

    function preload() {
        this.load.spritesheet('frog', 'img/frog.png', { 
            frameWidth: 16, 
            frameHeight: 16 
        });
    }

    function create() {
        const frog = this.add.sprite(14,14, 'frog');
        this.anims.create({
            key: 'jump',
            frames: this.anims.generateFrameNumbers('frog', { start: 7*14+6, end: 7*14+13 }),
            frameRate: 10,
            repeat: -1
        });
        frog.play('jump');
    }
}

