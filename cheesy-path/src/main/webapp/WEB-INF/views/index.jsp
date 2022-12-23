<%@taglib uri='http://java.sun.com/jsp/jstl/core' prefix='c'%>
<!DOCTYPE html>
<html>
    <head>
        <title>Cheesy Path</title>

        <script src='https://ajax.googleapis.com/ajax/libs/jquery/3.1.1/jquery.min.js'></script>
        <script src='https://ajax.googleapis.com/ajax/libs/jqueryui/1.12.1/jquery-ui.min.js'></script>
        <script type='text/javascript' src='<c:url value='/resources/js/script.js' />'></script>

        <link href="https://fonts.googleapis.com/icon?family=Material+Icons" rel="stylesheet">
        <link rel='shortcut icon' href='https://media.team254.com/homepage/icons/favicon32.png' />
        <link href='<c:url value='/resources/css/style.css' />' rel='stylesheet'>
    </head>
    <body onload='init()'>
        <input id='title' placeholder='Title' disabled>
        <div id='canvases'>
            <canvas id='background'></canvas>
            <canvas id='field'></canvas>
            <svg id='interactive'></svg>
        </div>
        <div class='buttonContainer'>
            <button onclick='addPoint()' class="icon-button" title="Add Point"><i class="material-icons">add</i></button>
            <button onclick='update(false)' class="icon-button" title="Refresh"><i class="material-icons">refresh</i></button>
            <button onclick='draw(3)' class="icon-button" title="Animate"><i class="material-icons">play_arrow</i></button>
            <button onclick="showWaypointsList()">Waypoints Code</button>
            <button id="resetButton" onclick="restoreFromFile()" class="icon-button" title="Restore from file"><i class="material-icons">sync</i></button>
            <button id="openButton" onclick="openFile()" class="btn-pair-left">Open</button>
            <button id="saveButton" onclick="saveFile()" class="btn-pair-center">
                Save
                <span class="modified-indicator">&bull;</span>
            </button>
            <button id="saveAsButton" onclick="saveFileAs()" class="btn-pair-right">Save As</button>
            <div class="spacer"></div>
            <select onchange='changeField(this.value)'>
                <option value="6_field1" selected>6_field1</option>
                <option value="6_field2">6_field2</option>
                <option value="7_field1">7_field1</option>
                <option value="7_field2">7_field2</option>
                <option value="7_field3">7_field3</option>
                <option value="snowremoval">Snow Removal</option>
                <option value="SnowThrower">Snow Thrower</option>
            </select>
            <label class='checkbox'>Is reversed: <input type='checkbox' class='data-input' id='isReversed'></label>
        </div>
        <table>
            <thead>
                <th></th>
                <th>X</th>
                <th>Y</th>
                <th>Heading</th>
                <th>Comments</th>
                <th>Enabled</th>
                <th>Delete</th>
            </thead>
            <tbody>
                <tr>
                    <td class='drag-handler'><i class="material-icons">drag_indicator</i></td>
                    <td class='x'><input type='number' class='data-input' value='0'></td>
                    <td class='y'><input type='number' class='data-input' value='0'></td>
                    <td class='heading'><input type='number' class='data-input' value='0'></td>
                    <td class='comments'><input type='search' placeholder='Comments'></td>
                    <td class='enabled'><input type='checkbox' class='data-input' checked></td>
                    <td class='delete'>
                        <button onclick='$(this).parent().parent().remove();update()' class="icon-button">
                            <i class='material-icons'>clear</i>
                        </button>
                    </td>
                </tr>
            </tbody>
        </table>
        <dialog id="waypointsDialog">
            <button onclick='this.parentElement.close()' class="close-button">&times;</button>
            <h3>Waypoints List</h3>
            <pre onclick="copyToClipboard()"><code id="waypointsOutput"></code></pre>
        </dialog>
        <div class="toast" id="clipboardToast">Copied to clipboard!</div>
    </body>
</html>

<script>
    $('table tbody').sortable({
        helper: fixWidthHelper,
        update: update,
        forcePlaceholderSize: true,
    }).disableSelection();

    function fixWidthHelper(e, ui) {
        ui.children().each(function() {
            $(this).width($(this).width());
        });
        return ui;
    }
</script>
