document.addEventListener("DOMContentLoaded", function () {
    const statusDiv = document.getElementById('status');
    const distancesEl = document.getElementById('distances');
    const clearDistancesBtn = document.getElementById('clearDistances');
    const objectsEl = document.getElementById('objects');
    const scaleBtn = document.getElementById('scaleBtn');

    let scale = 'far';

    const socket = io('http://localhost:4000');

    socket.on('connect', function () {
        statusDiv.textContent = 'Status: Connected';
    });

    socket.on('distances', function (data) {

        if (distancesEl.childElementCount >= 100) {
            return;
        }

        const distancesFormatted = data.distances.map(distance => distance.padStart(4, ' '));
        const listItem = document.createElement('li');
        listItem.textContent = distancesFormatted.join(', ');
        distancesEl.appendChild(listItem);

        objectsEl.innerHTML = '';
        const numOfObjectsFound = distancesFormatted.length;

        for (let i = 0; i < numOfObjectsFound; i++) {
            const objectEl = document.createElement('div');
            objectEl.classList.add('object');

            if (scale === 'far') {
                document.getElementById('ruler-1').textContent = '3m';
                document.getElementById('ruler-2').textContent = '2m';
                document.getElementById('ruler-3').textContent = '1m';
                document.getElementById('ruler-4').textContent = '0m';
                objectEl.style.bottom = data.distances[i] / 3000 * 100 + '%';
            }
            else {
                document.getElementById('ruler-1').textContent = '60cm';
                document.getElementById('ruler-2').textContent = '40cm';
                document.getElementById('ruler-3').textContent = '20cm';
                document.getElementById('ruler-4').textContent = '0cm';
                objectEl.style.bottom = data.distances[i] / 600 * 100 + '%';
            }
            
            objectsEl.appendChild(objectEl);
        }

        // Scroll to the bottom of the distancesEl
        distancesEl.scrollTop = distancesEl.scrollHeight;
    });

    socket.on('disconnect', function () {
        statusDiv.textContent = 'Status: Disconnected';
    });

    socket.on('connect_error', function (error) {
        statusDiv.textContent = 'Status: Connection Error';
        console.error('Connection Error:', error);
    });

    clearDistancesBtn.addEventListener('click', function () {
        distancesEl.innerHTML = '';
    });

    scaleBtn.addEventListener('click', function () {
        if (scale === 'far') {
            scale = 'close';
        }
        else {
            scale = 'far';
        }
    });
});