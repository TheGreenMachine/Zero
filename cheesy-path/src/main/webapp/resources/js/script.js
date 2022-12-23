let waypoints = [];
let splinePoints = [];
let fieldCanvas;
let ctx;
let ctxBackground;
let image;
let wto;
let change = "propertychange change input";
let animating = false;
let waypointsOutput;
let waypointsDialog;
let titleInput;
let interactive;
let clipboardToast;
let isReversedCheckbox;

const fieldWidth = 360; // inches
const fieldHeight = 180; // inches

const xOffset = 0; // inches
const yOffset = 0; // inches

const width = 980; //pixels
const height = 490; //pixels

const robotWidth = 34; // inches
const robotHeight = 34; // inches

const waypointRadius = 7;
const splineWidth = 3;

const kEps = 1E-9;
const pi = Math.PI;

function svg(tagName, attrs) {
    const svgNs = "http://www.w3.org/2000/svg";
    let element = document.createElementNS(svgNs, tagName);
    if (attrs && typeof attrs === 'object') {
        for (const [key, value] of Object.entries(attrs)) {
            element.setAttribute(key, value);
        }
    }
    return element;
}

class Translation2d {
	constructor(x, y) {
		this.x = x;
		this.y = y;
	}

	norm() {
        return Math.hypot(this.x, this.y);
    }

    norm2() {
        return this.x * this.x + this.y * this.y;
    }

    translateBy(other) {
        return new Translation2d(this.x + other.x, this.y + other.y);
    }

    rotateBy(rotation) {
        return new Translation2d(this.x * rotation.cos - this.y * rotation.sin, this.x * rotation.sin + this.y * rotation.cos);
    }

    direction() {
        return new Rotation2d(this.x, this.y, true);
    }

    inverse() {
		return new Translation2d(-this.x, -this.y);
    }

    interpolate(other, x) {
        if (x <= 0) {
            return new Translation2d(this.x, this.y);
        } else if (x >= 1) {
            return new Translation2d(other.x, other.y);
        }
        return this.extrapolate(other, x);
    }

    extrapolate(other, x) {
        return new Translation2d(x * (other.x - this.x) + this.x, x * (other.y - this.y) + this.y);
    }

	scale(s) {
		return new Translation2d(this.x * s, this.y * s);
    }

    static dot(a, b) {
		return a.x * b.x + a.y * b.y;
    }

    static getAngle(a, b) {
        let cos_angle = this.dot(a, b) / (a.norm() * b.norm());
        if (Double.isNaN(cos_angle)) {
            return new Rotation2d(1, 0, false);
        }

        return Rotation2d.fromRadians(Math.acos(Math.min(1.0, Math.max(cos_angle, -1.0))));
    }

    static cross(a, b) {
        return a.x * b.y - a.y * b.x;
    }

    distance(other) {
        return this.inverse().translateBy(other).norm();
    }

	draw(color, radius) {
		color = color || "#2CFF2C";
		ctx.beginPath();
		ctx.arc(this.drawX, this.drawY, radius, 0, 2 * Math.PI, false);
		ctx.fillStyle = color;
		ctx.strokeStyle = color;
		ctx.fill();
		ctx.lineWidth = 0;
		ctx.stroke();
	}

	get drawX() {
		return (this.x + xOffset) * (width / fieldWidth);
	}

	get drawY() {
		return height - (this.y + yOffset) * (height / fieldHeight);
	}
}

class Rotation2d {
	constructor(x, y, normalize) {
        this.cos = x;
        this.sin = y;
        this.normalize = normalize;
        if (normalize) {
            this.normalizeFunc();
        }
    }

    static fromRadians(angle_radians) {
        return new Rotation2d(Math.cos(angle_radians), Math.sin(angle_radians), false);
    }

    static fromDegrees(angle_degrees) {
        return this.fromRadians(d2r(angle_degrees));
    }

    normalizeFunc() {
        let magnitude = Math.hypot(this.cos, this.sin);
        if (magnitude > kEps) {
            this.cos /= magnitude;
            this.sin /= magnitude;
        } else {
            this.sin = 0;
            this.cos = 1;
        }
    }

    tan() {
        if (Math.abs(this.cos) < kEps) {
            if (this.sin >= 0.0) {
                return Number.POSITIVE_INFINITY;
            } else {
                return Number.NEGATIVE_INFINITY;
            }
        }
        return this.sin / this.cos;
    }

    getRadians() {
        return Math.atan2(this.sin, this.cos);
    }

    getDegrees() {
        return r2d(this.getRadians());
    }

    rotateBy(other) {
        return new Rotation2d(this.cos * other.cos - this.sin * other.sin,
                this.cos * other.sin + this.sin * other.cos, true);
    }

    normal() {
        return new Rotation2d(-this.sin, this.cos, false);
    }

    inverse() {
        return new Rotation2d(this.cos, -this.sin, false);
    }

    interpolate(other, x) {
        if (x <= 0) {
            return new Rotation2d(this.cos, this.sin, this.normalize);
        } else if (x >= 1) {
            return new Rotation2d(other.cos, other.sin, other.normalize);
        }
        let angle_diff = this.inverse().rotateBy(other).getRadians();
        return this.rotateBy(Rotation2d.fromRadians(angle_diff * x));
    }

    distance(other) {
        return this.inverse().rotateBy(other).getRadians();
    }
}

class Pose2d {
	constructor(translation, rotation, comment) {
		this.translation = translation;
		this.rotation = rotation;
        this.comment = comment || "";
    }

	static exp(delta) {
        let sin_theta = Math.sin(delta.dtheta);
        let cos_theta = Math.cos(delta.dtheta);
        let s, c;

        if (Math.abs(delta.dtheta) < kEps) {
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = .5 * delta.dtheta;
        } else {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }

        return new Pose2d(new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                new Rotation2d(cos_theta, sin_theta, false));
    }

    static log(transform) {
        let dtheta = transform.getRotation().getRadians();
        let half_dtheta = 0.5 * dtheta;
        let cos_minus_one = transform.getRotation().cos() - 1.0;
        let halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().sin()) / cos_minus_one;
        }
        let translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
        return new Twist2d(translation_part.x(), translation_part.y(), dtheta);
    }

    get getTranslation() {
        return this.translation;
    }

    get getRotation() {
        return this.rotation;
    }

    transformBy(other) {
        return new Pose2d(this.translation.translateBy(other.translation.rotateBy(this.rotation)),
                this.rotation.rotateBy(other.rotation));
    }

    inverse() {
        let rotation_inverted = this.rotation.inverse();
        return new Pose2d(this.translation.inverse().rotateBy(rotation_inverted), rotation_inverted);
    }

    normal() {
        return new Pose2d(this.translation, this.rotation.normal());
    }

    interpolate(other, x) {
        if (x <= 0) {
            return new Pose2d(this.translation, this.rotation, this.comment);
        } else if (x >= 1) {
            return new Pose2d(other.translation, other.rotation, other.comment);
        }
        let twist = Pose2d.log(this.inverse().transformBy(other));
        return this.transformBy(Pose2d.exp(twist.scaled(x)));
    }

    distance(other) {
        return Pose2d.log(this.inverse().transformBy(other)).norm();
    }

    heading(other) {
	    return Math.atan2(this.translation.y - other.translation.y, this.translation.x - other.translation.x);
    }

    draw(drawHeading, radius) {
		this.translation.draw(null, radius);

		if (!drawHeading) {
		    return;
        }

        let x = this.translation.drawX;
        let y = this.translation.drawY;

		ctx.beginPath();
		ctx.moveTo(x, y);
		ctx.lineTo(x + 25 * Math.cos(-this.rotation.getRadians()), y + 25 * Math.sin(-this.rotation.getRadians()));
		ctx.lineWidth = 3;
		ctx.stroke();
        ctx.closePath();
	}

	drawInteractive(radius, index) {
	    let point = svg('circle', {
            fill: "#2CFF2C",
            cx: this.translation.drawX,
            cy: this.translation.drawY,
            r: radius,
            'data-index': index,
        });

	    point.addEventListener('mousedown', handleWaypointDragStart);
        point.addEventListener('click', handleWaypointClick);

        interactive.appendChild(point);
    }

	toString() {
		return "new Pose2d(new Translation2d(" + this.translation.x + ", " + this.translation.y + "), new Rotation2d(" + this.rotation.cos + ", " + this.rotation.sin + ", " + this.rotation.normalize + "))";
	}

    transform(other) {
        other.position.rotate(this.rotation);
        this.translation.translate(other.translation);
        this.rotation.rotate(other.rotation);
    }
}

function d2r(d) {
    return d * (Math.PI / 180);
}

function r2d(r) {
    return r * (180 / Math.PI);
}

function fillRobot(position, heading, color) {
    let previous = ctx.globalCompositeOperation;
    ctx.globalCompositeOperation = "destination-over";

    let translation = position.translation;

    ctx.translate(translation.drawX, translation.drawY);
    ctx.rotate(-heading);

    let w = robotWidth * (width / fieldWidth);
    let h = robotHeight * (height / fieldHeight);
    ctx.fillStyle = color || "rgba(0, 0, 0, 0)";
    ctx.fillRect(-h / 2, -w / 2, h, w);

    ctx.rotate(heading);
    ctx.translate(-translation.drawX, -translation.drawY);

    ctx.globalCompositeOperation = previous;
}

let r = Math.sqrt(Math.pow(robotWidth, 2) + Math.pow(robotHeight, 2)) / 2;
let t = Math.atan2(robotHeight, robotWidth);

function drawRobot(position, heading) {
    let h = heading;
    let angles = [h + (pi / 2) + t, h - (pi / 2) + t, h + (pi / 2) - t, h - (pi / 2) - t];

    let points = [];

    angles.forEach(function(angle) {
        let point = new Translation2d(position.translation.x + (r * Math.cos(angle)),
            position.translation.y + (r * Math.sin(angle)));
        points.push(point);
        point.draw(Math.abs(angle - heading) < pi / 2 ? "#00AAFF" : "#0066FF", splineWidth);
    });
}

function init() {
    let field = $('#field');
    let background = $('#background');
    let canvases = $('#canvases');
    let interactiveEl = $('#interactive');
    let widthString = (width / 1.5) + "px";
    let heightString = (height / 1.5) + "px";

	field.css("width", widthString);
    field.css("height", heightString);
    background.css("width", widthString);
    background.css("height", heightString);
    interactiveEl.css("width", widthString);
    interactiveEl.css("height", heightString);
    canvases.css("width", widthString);
    canvases.css("height", heightString);
    fieldCanvas = document.getElementById('field');

	ctx = fieldCanvas.getContext('2d');
	ctx.canvas.width = width;
	ctx.canvas.height = height;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#FF0000";

    ctxBackground = document.getElementById('background').getContext('2d');
    ctxBackground.canvas.width = width;
    ctxBackground.canvas.height = height;
    ctx.clearRect(0, 0, width, height);

    interactive = document.getElementById('interactive');
    interactive.setAttribute("width", width);
    interactive.setAttribute("height", height);
    interactive.setAttribute("viewBox", `0 0 ${width} ${height}`);
    interactive.addEventListener('click', onCanvasClick);

	image = new Image();
	image.src = 'resources/img/6_field1.jpg';
	image.onload = function() {
		ctxBackground.drawImage(image, 0, 0, width, height);
		update(false);
	};

	titleInput = document.getElementById("title");

    isReversedCheckbox = document.getElementById('isReversed');
    waypointsDialog = document.getElementById('waypointsDialog');
    waypointsOutput = document.getElementById('waypointsOutput');
    clipboardToast = document.getElementById('clipboardToast');

    document.addEventListener('keydown', (e) => {
        if (e.code === 'KeyS' && (e.ctrlKey || e.metaKey)) {
            e.preventDefault();
            saveFile();
        }
    })

    rebind();
}

function clearSplines() {
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#FF0000";
}

function clear() {
    clearSplines();

	ctxBackground.clearRect(0, 0, width, height);
    ctxBackground.fillStyle = "#FF0000";
    ctxBackground.drawImage(image, 0, 0, width, height);

    while (interactive.lastChild) {
        interactive.removeChild(interactive.lastChild);
    }
}

function rebind() {
    let input = $('.data-input');
    input.unbind(change);
    input.bind(change, function() {
        cancelAnimationFrame(wto);
        wto = requestAnimationFrame(function() {
            update();
        });
    });
}

function addPoint() {
	let prev;
	if (waypoints.length > 0) prev = waypoints[waypoints.length - 1].translation;
	else prev = new Translation2d(50, 50);
	_addPoint(prev.x + 50, prev.y + 50);
}

function _addPoint(x, y, heading = 0, doUpdate = true) {
    $("tbody").append("<tr>" + "<td class='drag-handler'><i class='material-icons'>drag_indicator</i></td>"
        + `<td class='x'><input type='number' class='data-input' value='${x}'></td>`
        + `<td class='y'><input type='number' class='data-input' value='${y}'></td>`
        + `<td class='heading'><input type='number' class='data-input' value='${heading}'></td>`
        + "<td class='comments'><input type='search' placeholder='Comments'></td>"
        + "<td class='enabled'><input type='checkbox' class='data-input' checked></td>"
        + "<td class='delete'><button onclick='$(this).parent().parent().remove();update()' class='icon-button'><i class='material-icons'>clear</i></button></td></tr>");
    if (doUpdate) {
        update();
        rebind();
    }
}

function getCursorPosition(event) {
    const rect = interactive.getBoundingClientRect();
    return {
        x: (event.clientX - rect.left) * (width / rect.width),
        y: (event.clientY - rect.top) * (height / rect.height),
    };
}

function onCanvasClick(event) {
    let { x: canvasX, y: canvasY } = getCursorPosition(event);
    let { x, y } = canvasToFieldCoords(canvasX, canvasY);
    _addPoint(x, y);
}

function canvasToFieldCoords(canvasX, canvasY) {
    let x = Math.round(canvasX * (fieldWidth / width) - xOffset);
    let y = Math.round((height - canvasY) * (fieldHeight / height) - yOffset);
    return { x, y };
}

let selectedWaypoint;
function selectWaypoint(el) {
    if (el === selectedWaypoint) return;
    if (selectedWaypoint) {
        selectedWaypoint.removeAttribute('data-selected');
    }
    selectedWaypoint = el;
    if (selectedWaypoint) {
        selectedWaypoint.setAttribute('data-selected', true);
    }
}

function handleWaypointDragStart(event) {
    selectWaypoint(event.target);
    fieldCanvas.classList.add('faded');
    interactive.addEventListener('mousemove', handleWaypointDrag);
    interactive.addEventListener('mouseup', handleWaypointDragEnd);
}

function handleWaypointDrag(event) {
    if (selectedWaypoint) {
        event.preventDefault();
        let { x: canvasX, y: canvasY } = getCursorPosition(event);
        selectedWaypoint.setAttribute("cx", canvasX);
        selectedWaypoint.setAttribute("cy", canvasY);
        let index = selectedWaypoint.getAttribute('data-index');
        let { x, y } = canvasToFieldCoords(canvasX, canvasY);
        waypoints[index].translation.x = x;
        waypoints[index].translation.y = y;

        recalculateSplines(waypoints, 4);
    }
}

function handleWaypointClick(event) {
    event.stopPropagation();
}

function handleWaypointDragEnd(event) {
    if (selectedWaypoint) {
        let { x: canvasX, y: canvasY } = getCursorPosition(event);
        let { x, y } = canvasToFieldCoords(canvasX, canvasY);
        modifyWaypoint(selectedWaypoint.getAttribute('data-index'), x, y);
        selectWaypoint(null);
    }
    fieldCanvas.classList.remove('faded');
    interactive.removeEventListener('mousemove', handleWaypointDrag);
    interactive.removeEventListener('mouseup', handleWaypointDragEnd);
}

function modifyWaypoint(index, x, y) {
    let tr = $('tbody').children('tr')[index];
    let xInput = tr.querySelector('.x input');
    let yInput = tr.querySelector('.y input');

    xInput.value = x;
    yInput.value = y;

    update();
    rebind();
}

function draw(style) {
    if (style === 4) {
        clearSplines();
        drawSplines(true);
        drawSplines(false);
        return;
    }
    clear();
    drawWaypoints();

    switch (style) {
        // waypoints only
        case 1:
            break;
        // all
        case 2:
            drawSplines(true);
            drawSplines(false);
            break;
        case 3:
            animate();
            break;
    }
}

function update(modified = true) {
    if (animating) {
        return;
    }

	waypoints = [];
	let data = "";
	$('tbody').children('tr').each(function() {
		let x = parseInt($($($(this).children()).children()[1]).val());
		let y = parseInt($($($(this).children()).children()[2]).val());
		let heading = Math.round(parseInt($($($(this).children()).children()[3]).val()));
		if (isNaN(heading)) {
			heading = 0;
        }
		let comment = ($($($(this).children()).children()[4]).val());
        let enabled = ($($($(this).children()).children()[5]).prop('checked'));
		if (enabled) {
            waypoints.push(new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(heading), comment));
        }
    });

    draw(1);

    if (modified) {
        setModified(true);
    }

    recalculateSplines(waypoints, 2);
}

function recalculateSplines(waypointsList, drawStyle) {
    const orderedWaypoints = isReversedCheckbox.checked ? waypointsList.slice(0).reverse() : waypointsList;
    const data = orderedWaypoints.map(point => (
        `${point.translation.x},${point.translation.y},${Math.round(point.rotation.getDegrees())}`
    )).join(';');

    if (data.length !== 0) {
        $.post({
            url: "api/calculate_splines",
            data: data,
            success: function (splineData) {
                if (splineData === "no") {
                    return;
                }

                // console.log(splineData);

                let points = JSON.parse(splineData).points;

                splinePoints = [];
                for (const point of points) {
                    splinePoints.push(new Pose2d(new Translation2d(point.x, point.y), Rotation2d.fromRadians(point.rotation)));
                }

                draw(drawStyle);
            }
        });
    }
}

function changeField(val) {
    console.log(val);
	image.src = `resources/img/${val}.jpg`
    image.onload(() => {
        ctx.drawImage(image, 0, 0, width, height);
        update(false);
    });
}

function drawWaypoints() {
	waypoints.forEach((waypoint, i) => {
        waypoint.drawInteractive(waypointRadius, i);
        drawRobot(waypoint, waypoint.rotation.getRadians());
    });
}

let animation;

function animate() {
    drawSplines(false, true);
}

function drawSplines(fill, animate) {
    animate = animate || false;
    let i = 0;

    if (animate) {
        clearInterval(animation);

        animation = setInterval(function() {
            if (i === splinePoints.length) {
                animating = false;
                clearInterval(animation);
                return;
            }

            animating = true;

            let splinePoint = splinePoints[i];
            let hue = Math.round(180 * (i++ / splinePoints.length));

            let previous = ctx.globalCompositeOperation;
            fillRobot(splinePoint, splinePoint.rotation.getRadians(), 'hsla(' + hue + ', 100%, 50%, 0.025)');
            ctx.globalCompositeOperation = "source-over";
            drawRobot(splinePoint, splinePoint.rotation.getRadians());
            splinePoint.draw(false, splineWidth);
            ctx.globalCompositeOperation = previous;
        }, 25);
    } else {
        splinePoints.forEach(function(splinePoint) {
            splinePoint.draw(false, splineWidth);

            if (fill) {
                let index = isReversedCheckbox.checked ? (splinePoints.length - i++) : i++;
                let hue = Math.round(180 * (index / splinePoints.length));
                fillRobot(splinePoint, splinePoint.rotation.getRadians(), 'hsla(' + hue + ', 100%, 50%, 0.025)');
            } else {
                drawRobot(splinePoint, splinePoint.rotation.getRadians());
            }
        });
    }
}

function showWaypointsList() {
    waypointsOutput.textContent = generateWaypointsList();
    waypointsDialog.showModal();
}

async function copyToClipboard() {
    let range = new Range();
    range.selectNode(waypointsOutput);
    window.getSelection().empty();
    window.getSelection().addRange(range);
    await navigator.clipboard.writeText(waypointsOutput.textContent);
    showToast(clipboardToast);
}

const TOAST_DURATION = 1000; // ms

function showToast(toastEl) {
    toastEl.classList.add('shown');
    setTimeout(() => {
        toastEl.classList.remove('shown');
    }, TOAST_DURATION);
}

function generateWaypointsList() {
    return 'List.of(\n' +
        waypoints.map((waypoint, i, arr) =>
            `\tnew Pose2d(${waypoint.translation.x}, ${waypoint.translation.y}, ${Math.round(waypoint.rotation.getDegrees())})`
            + (i === arr.length - 1 ? '' : ',')
            + (waypoint.comment && ` // ${waypoint.comment}`)
        ).join('\n') +
        '\n)';
}

function loadWaypoints(data) {
    waypoints = [];
    $('tbody').empty();
    for (const {x, y, heading} of data) {
        _addPoint(x, y, heading, false);
    }
    update(false);
    rebind();
}

class CSV {
    constructor(data = [], isReversed) {
        this.data = data;
        this.isReversed = isReversed;
    }

    static load(text) {
        const rows = text.split("\n");
        const headers = rows.shift(); // gets and removes headers
        const reversedText = headers.split(',')[3]?.trim();
        const reversed = !!(reversedText && reversedText === 'true'); // ignore truthy values, explicit true
        const data = rows.map(row => {
            const [ x, y, heading ] = row.split(",");
            return { x, y, heading };
        });
        return new CSV(data, reversed);
    }

    addRow({ x, y, heading }) {
        this.data.push({ x, y, heading });
    }

    toString() {
        let returnVal = `x,y,heading,${this.isReversed}\n`;
        returnVal += this.data.map(({x, y, heading}) => `${x},${y},${heading}`).join('\n');
        return returnVal;
    }

    toBlob() {
        return new Blob([this.toString()], { type: 'text/csv' });
    }
}

function setModified(modified) {
    if (modified) {
        document.documentElement.setAttribute('data-modified', 'true');
    } else {
        document.documentElement.removeAttribute('data-modified');
    }
}

const filePickerOptions = {
    types: [
        {
            description: 'CSV Files',
            accept: {
                'text/csv': ['.csv'],
            },
        },
    ],
};
let fileHandle;

async function openFile() {
    [fileHandle] = await window.showOpenFilePicker(filePickerOptions);
    const file = await fileHandle.getFile();
    await loadFromFile(file);
}

async function restoreFromFile() {
    if (fileHandle) {
        const file = await fileHandle.getFile();
        await loadFromFile(file);
    }
}

async function loadFromFile(file) {
    titleInput.value = file.name.slice(0, -4);
    const text = await file.text();
    const output = CSV.load(text);
    isReversedCheckbox.checked = output.isReversed;
    loadWaypoints(output.data);
}

async function writeFile(fileHandle, contents) {
    const writable = await fileHandle.createWritable();
    await writable.write(contents);
    await writable.close();
}

function generateCSV() {
    const csv = new CSV(
        waypoints.map(point => ({
            x: point.translation.x,
            y: point.translation.y,
            heading: Math.round(point.rotation.getDegrees()),
        })),
        isReversedCheckbox.checked
    );
    return csv.toString();
}

async function saveFile() {
    try {
        if (!fileHandle) {
            return await saveFileAs();
        }
        await writeFile(fileHandle, generateCSV());
    } catch (e) {
        console.error('Unable to save file', e);
    }
    setModified(false);
}

async function saveFileAs() {
    try {
        fileHandle = await window.showSaveFilePicker(filePickerOptions);
        titleInput.value = fileHandle.name.slice(0, -4);
    } catch (e) {
        if (e.name === 'AbortError') return;
        console.error('An error occurred trying to open the file', e);
        return;
    }
    try {
        await writeFile(fileHandle, generateCSV());
    } catch (e) {
        console.error('Unable to save file', e);
    }
    setModified(false);
}
