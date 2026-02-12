// Global variables
let geojsonData = null;
let svg, g, tileLayer, projection, path, tooltip;
let width, height;
let currentTransform = d3.zoomIdentity;
let currentFilter = '';
let highlightedRoadId = null;
let tile = d3.tile();

// Initialize
function init() {
    width = document.getElementById('map-container').clientWidth;
    height = document.getElementById('map-container').clientHeight;

    // Create SVG
    svg = d3.select('#map-container')
        .append('svg')
        .attr('width', width)
        .attr('height', height);

    // Create tile layer group (background)
    tileLayer = svg.append('g')
        .attr('class', 'tile-layer');

    // Create main group for roads (foreground)
    g = svg.append('g');

    // Set projection - suitable for Martinez, CA
    projection = d3.geoMercator()
        .center([-122.1267, 38.0025]) // Coordinates for Martinez, CA
        .scale(100000) // Increased initial scale for better visibility
        .translate([width / 2, height / 2]);

    path = d3.geoPath().projection(projection);

    // Set up zoom behavior
    const zoom = d3.zoom()
        .scaleExtent([0.1, 50]) // Increased max zoom for better detail
        .on('zoom', (event) => {
            currentTransform = event.transform;
            updateTiles(event.transform);
            g.attr('transform', event.transform);
            updateZoomLevel(event.transform.k);
        });

    svg.call(zoom);

    // Tooltip
    tooltip = d3.select('#tooltip');

    // Initialize tiles
    updateTiles(d3.zoomIdentity);

    // Load data
    loadGeoJSON();

    // Bind control events
    setupControls();
}

// Update map tiles
function updateTiles(transform) {
    if (!tileLayer || !projection) return;

    try {
        // Use d3-tile if available
        if (typeof d3.tile !== 'undefined') {
            // Configure tile function - tiles use Web Mercator projection
            const tileProj = d3.geoMercator();
            
            tile
                .size([width, height])
                .scale(transform.k * projection.scale())
                .translate([transform.x + width / 2, transform.y + height / 2])
                .wrap(false);

            // Get tiles for current view
            const tiles = tile();

            if (!tiles || tiles.length === 0) {
                updateTilesManual(transform);
                return;
            }

            // Update tile images
            const tileImages = tileLayer
                .selectAll('image')
                .data(tiles, d => `${d[0]}-${d[1]}-${d[2]}`);

            // Remove old tiles
            tileImages.exit().remove();

            // Add new tiles
            const tileImagesEnter = tileImages.enter()
                .append('image')
                .attr('class', 'tile')
                .attr('xlink:href', d => {
                    // Use OpenStreetMap tile server
                    const servers = ['a', 'b', 'c'];
                    const server = servers[(d[0] + d[1] + d[2]) % 3];
                    return `https://${server}.tile.openstreetmap.org/${d[2]}/${d[0]}/${d[1]}.png`;
                })
                .attr('x', d => (d[0] + tiles.translate[0]) * tiles.scale)
                .attr('y', d => (d[1] + tiles.translate[1]) * tiles.scale)
                .attr('width', tiles.scale)
                .attr('height', tiles.scale)
                .on('error', function() {
                    d3.select(this).style('display', 'none');
                });

            // Update existing tiles
            tileImages.merge(tileImagesEnter)
                .attr('x', d => (d[0] + tiles.translate[0]) * tiles.scale)
                .attr('y', d => (d[1] + tiles.translate[1]) * tiles.scale)
                .attr('width', tiles.scale)
                .attr('height', tiles.scale);
        } else {
            // Fallback to manual calculation
            updateTilesManual(transform);
        }
    } catch (error) {
        console.warn('Error updating tiles with d3-tile:', error);
        updateTilesManual(transform);
    }
}

// Manual tile update using Web Mercator projection
function updateTilesManual(transform) {
    if (!tileLayer || !projection) return;

    // Clear existing tiles
    tileLayer.selectAll('image').remove();

    // Calculate zoom level based on current scale
    const k = transform.k;
    const baseScale = projection.scale();
    const actualScale = k * baseScale;
    
    // Web Mercator: scale = 256 * 2^zoom / (2 * PI)
    // Solving for zoom: zoom = log2(scale * 2 * PI / 256)
    const zoomLevel = Math.min(Math.max(Math.floor(Math.log2(actualScale * 2 * Math.PI / 256)), 0), 18);
    const n = Math.pow(2, zoomLevel);

    // Get viewport corners in geographic coordinates (accounting for transform)
    const topLeftGeo = projection.invert([
        (-transform.x) / k,
        (-transform.y) / k
    ]);
    const bottomRightGeo = projection.invert([
        (width - transform.x) / k,
        (height - transform.y) / k
    ]);
    
    if (!topLeftGeo || !bottomRightGeo) return;

    // Convert to Web Mercator tile coordinates
    function lon2tile(lon, zoom) {
        return Math.floor((lon + 180) / 360 * Math.pow(2, zoom));
    }
    
    function lat2tile(lat, zoom) {
        return Math.floor((1 - Math.log(Math.tan(lat * Math.PI / 180) + 1 / Math.cos(lat * Math.PI / 180)) / Math.PI) / 2 * Math.pow(2, zoom));
    }

    const minX = Math.max(0, lon2tile(Math.min(topLeftGeo[0], bottomRightGeo[0]), zoomLevel) - 1);
    const maxX = Math.min(n - 1, lon2tile(Math.max(topLeftGeo[0], bottomRightGeo[0]), zoomLevel) + 1);
    const minY = Math.max(0, lat2tile(Math.max(topLeftGeo[1], bottomRightGeo[1]), zoomLevel) - 1);
    const maxY = Math.min(n - 1, lat2tile(Math.min(topLeftGeo[1], bottomRightGeo[1]), zoomLevel) + 1);

    // Create Web Mercator projection matching OSM tiles
    const webMercator = d3.geoMercator()
        .scale(n * 256 / (2 * Math.PI))
        .translate([n * 256 / 2, n * 256 / 2]);

    // Add tiles
    for (let x = minX; x <= maxX; x++) {
        for (let y = minY; y <= maxY; y++) {
            // Calculate tile's geographic bounds
            const tileLonMin = (x / n) * 360 - 180;
            const tileLonMax = ((x + 1) / n) * 360 - 180;
            const tileLatMax = Math.atan(Math.sinh(Math.PI * (1 - 2 * y / n))) * 180 / Math.PI;
            const tileLatMin = Math.atan(Math.sinh(Math.PI * (1 - 2 * (y + 1) / n))) * 180 / Math.PI;
            
            // Get tile corners in Web Mercator pixel coordinates
            const topLeftPx = webMercator([tileLonMin, tileLatMax]);
            const bottomRightPx = webMercator([tileLonMax, tileLatMin]);

            // Convert to our projection's coordinates
            const topLeftProj = projection([tileLonMin, tileLatMax]);
            const bottomRightProj = projection([tileLonMax, tileLatMin]);
            
            if (!topLeftProj || !bottomRightProj) continue;

            // Apply transform and calculate SVG position
            const svgX = topLeftProj[0] * k + transform.x;
            const svgY = topLeftProj[1] * k + transform.y;
            const svgWidth = (bottomRightProj[0] - topLeftProj[0]) * k;
            const svgHeight = (bottomRightProj[1] - topLeftProj[1]) * k;

            tileLayer.append('image')
                .attr('class', 'tile')
                .attr('xlink:href', `https://a.tile.openstreetmap.org/${zoomLevel}/${x}/${y}.png`)
                .attr('x', svgX)
                .attr('y', svgY)
                .attr('width', Math.abs(svgWidth))
                .attr('height', Math.abs(svgHeight))
                .on('error', function() {
                    d3.select(this).style('display', 'none');
                });
        }
    }
}

// Load GeoJSON data
async function loadGeoJSON() {
    try {
        const response = await fetch('../ca_martinez.geojson');
        const data = await response.json();
        
        // Handle data format (if features are in data.features)
        geojsonData = data.features || data;
        
        document.getElementById('loading').style.display = 'none';
        
        // Update statistics
        updateStats();
        
        // Populate type filter dropdown
        populateTypeFilter();
        
        // Render map
        renderMap();
        
        // Auto fit bounds
        fitBounds();
    } catch (error) {
        console.error('Failed to load GeoJSON data:', error);
        document.getElementById('loading').innerHTML = 
            '<div style="color: #e74c3c;">❌ Load failed: ' + error.message + '</div>';
    }
}

// Render map
function renderMap() {
    if (!geojsonData) return;

    // Filter data based on current filter
    const filteredData = geojsonData.filter(d => {
        const props = d.properties || {};
        const typeName = props.type_names || '';
        const matchesFilter = !currentFilter || typeName === currentFilter;
        return matchesFilter;
    });

    // Use simplified strategy due to large dataset
    const roads = g.selectAll('.road')
        .data(filteredData, d => d.properties?.id || d.id);

    // Remove old roads
    roads.exit().remove();

    // Add new roads
    const roadsEnter = roads.enter()
        .append('path')
        .attr('class', 'road')
        .attr('d', path)
        .on('mouseover', function(event, d) {
            showTooltip(event, d);
            if (!d3.select(this).classed('highlighted')) {
                d3.select(this)
                    .attr('stroke-width', 2)
                    .attr('opacity', 1)
                    .attr('stroke', '#e74c3c');
            }
        })
        .on('mouseout', function() {
            hideTooltip();
            if (!d3.select(this).classed('highlighted')) {
                updateRoadStyle(d3.select(this));
            }
        })
        .on('mousemove', function(event) {
            moveTooltip(event);
        });

    // Update existing roads
    roads.merge(roadsEnter)
        .attr('d', path)
        .classed('filtered-out', false)
        .each(function(d) {
            updateRoadStyle(d3.select(this), d);
        });

    // Update filtered out roads
    g.selectAll('.road')
        .filter(function(d) {
            const props = d.properties || {};
            const typeName = props.type_names || '';
            return currentFilter && typeName !== currentFilter;
        })
        .classed('filtered-out', true)
        .attr('opacity', 0.1)
        .attr('stroke-width', 0.3);
}

// Update road style
function updateRoadStyle(selection, d) {
    const props = d?.properties || {};
    const roadId = props.id || d?.id;
    const isHighlighted = highlightedRoadId && roadId === highlightedRoadId;
    
    if (isHighlighted) {
        selection
            .classed('highlighted', true)
            .attr('stroke', '#f39c12')
            .attr('stroke-width', 3)
            .attr('opacity', 1);
    } else {
        selection
            .classed('highlighted', false)
            .attr('stroke', '#4a90e2')
            .attr('stroke-width', parseFloat(document.getElementById('strokeWidth').value))
            .attr('opacity', parseFloat(document.getElementById('opacity').value));
    }
}

// Show tooltip
function showTooltip(event, d) {
    const props = d.properties || {};
    const html = `
        <h3>Road Information</h3>
        <p><strong>ID:</strong> ${props.id || 'N/A'}</p>
        <p><strong>Length:</strong> ${props.length ? props.length.toFixed(2) + ' meters' : 'N/A'}</p>
        <p><strong>Type:</strong> ${props.type_names || 'N/A'}</p>
        <p><strong>Layer:</strong> ${props.layer || 'N/A'}</p>
        ${props.left_lane_width >= 0 ? `<p><strong>Left Lane Width:</strong> ${props.left_lane_width} meters</p>` : ''}
        ${props.right_lane_width >= 0 ? `<p><strong>Right Lane Width:</strong> ${props.right_lane_width} meters</p>` : ''}
    `;
    
    tooltip
        .html(html)
        .style('display', 'block');
    
    moveTooltip(event);
}

// Move tooltip
function moveTooltip(event) {
    tooltip
        .style('left', (event.pageX + 10) + 'px')
        .style('top', (event.pageY - 10) + 'px');
}

// Hide tooltip
function hideTooltip() {
    tooltip.style('display', 'none');
}

// Update statistics
function updateStats() {
    if (!geojsonData) return;

    // Count visible roads (after filtering)
    const visibleData = geojsonData.filter(d => {
        const props = d.properties || {};
        const typeName = props.type_names || '';
        return !currentFilter || typeName === currentFilter;
    });

    const roadCount = visibleData.length;
    const totalLength = visibleData.reduce((sum, d) => {
        return sum + (d.properties?.length || 0);
    }, 0);

    document.getElementById('roadCount').textContent = roadCount.toLocaleString();
    document.getElementById('totalLength').textContent = totalLength.toFixed(0).toLocaleString();
}

// Populate type filter dropdown
function populateTypeFilter() {
    if (!geojsonData) return;

    const typeSet = new Set();
    geojsonData.forEach(d => {
        const typeName = d.properties?.type_names;
        if (typeName) {
            typeSet.add(typeName);
        }
    });

    const typeArray = Array.from(typeSet).sort();
    const select = document.getElementById('typeFilter');
    
    // Clear existing options except "All Types"
    select.innerHTML = '<option value="">All Types</option>';
    
    // Add type options
    typeArray.forEach(type => {
        const option = document.createElement('option');
        option.value = type;
        option.textContent = type;
        select.appendChild(option);
    });
}

// Filter by type
function filterByType(typeName) {
    currentFilter = typeName;
    renderMap();
    updateStats();
}

// Search by ID
function searchById(roadId) {
    if (!geojsonData || !roadId) return null;

    const found = geojsonData.find(d => {
        const props = d.properties || {};
        return props.id === roadId || d.id === roadId;
    });

    if (found) {
        highlightedRoadId = found.properties?.id || found.id;
        
        // Update filter to show this road's type
        const typeName = found.properties?.type_names || '';
        if (typeName) {
            document.getElementById('typeFilter').value = typeName;
            currentFilter = typeName;
        }
        
        // Re-render map with highlight
        renderMap();
        updateStats();
        
        // Zoom to the road
        zoomToRoad(found);
        
        return found;
    } else {
        alert('Road with ID "' + roadId + '" not found.');
        return null;
    }
}

// Zoom to specific road
function zoomToRoad(road) {
    if (!road || !road.geometry) return;

    const bounds = d3.geoBounds(road);
    const [[lon0, lat0], [lon1, lat1]] = bounds;

    // Convert geographic bounds to pixel coordinates
    const p0 = projection([lon0, lat0]);
    const p1 = projection([lon1, lat1]);
    
    if (!p0 || !p1) return;

    const dx = p1[0] - p0[0];
    const dy = p1[1] - p0[1];
    const x = (p0[0] + p1[0]) / 2;
    const y = (p0[1] + p1[1]) / 2;
    
    // Add padding (0.7 = 30% padding for better view)
    const padding = 0.7;
    const scale = Math.min(width / dx, height / dy) * padding;
    const translate = [width / 2 - scale * x, height / 2 - scale * y];

    const transform = d3.zoomIdentity
        .translate(translate[0], translate[1])
        .scale(scale);

    svg.transition()
        .duration(1000)
        .call(d3.zoom().transform, transform)
        .on('end', () => {
            // Update tiles after transition
            updateTiles(transform);
        });
}

// Clear search highlight
function clearSearch() {
    highlightedRoadId = null;
    document.getElementById('idSearch').value = '';
    renderMap();
}

// Fit bounds
function fitBounds() {
    if (!geojsonData || geojsonData.length === 0) return;

    // Use filtered data if filter is active
    const dataToFit = currentFilter 
        ? geojsonData.filter(d => {
            const props = d.properties || {};
            const typeName = props.type_names || '';
            return typeName === currentFilter;
        })
        : geojsonData;

    if (dataToFit.length === 0) return;

    // Calculate geographic bounding box
    const bounds = d3.geoBounds({type: 'FeatureCollection', features: dataToFit});
    const [[lon0, lat0], [lon1, lat1]] = bounds;

    // Calculate center point
    const centerLon = (lon0 + lon1) / 2;
    const centerLat = (lat0 + lat1) / 2;

    // Update projection center
    projection.center([centerLon, centerLat]);

    // Convert geographic bounds to pixel coordinates using current projection
    const p0 = projection([lon0, lat0]);
    const p1 = projection([lon1, lat1]);
    
    if (!p0 || !p1) return;

    // Calculate appropriate scale and translation
    const dx = Math.abs(p1[0] - p0[0]);
    const dy = Math.abs(p1[1] - p0[1]);
    
    // Calculate scale with padding (0.8 = 20% padding for better view)
    const padding = 0.8;
    const scale = Math.min(width / dx, height / dy) * padding;
    
    // Get center point in pixel coordinates
    const centerPx = projection([centerLon, centerLat]);
    const translate = [width / 2 - scale * centerPx[0], height / 2 - scale * centerPx[1]];

    // Apply transformation
    const transform = d3.zoomIdentity
        .translate(translate[0], translate[1])
        .scale(scale);

    svg.transition()
        .duration(750)
        .call(d3.zoom().transform, transform)
        .on('end', () => {
            // Update tiles after transition
            updateTiles(transform);
        });
}

// Reset view
function resetView() {
    const identity = d3.zoomIdentity;
    svg.transition()
        .duration(750)
        .call(d3.zoom().transform, identity)
        .on('end', () => {
            // Update tiles after transition
            updateTiles(identity);
        });
}

// Update zoom level display
function updateZoomLevel(scale) {
    document.getElementById('zoomLevel').textContent = scale.toFixed(2) + 'x';
}

// Setup controls
function setupControls() {
    // Road width control
    d3.select('#strokeWidth').on('input', function() {
        const value = this.value;
        document.getElementById('strokeWidthValue').textContent = value;
        g.selectAll('.road:not(.highlighted)').attr('stroke-width', value);
    });

    // Opacity control
    d3.select('#opacity').on('input', function() {
        const value = this.value;
        document.getElementById('opacityValue').textContent = value;
        g.selectAll('.road:not(.highlighted)').attr('opacity', value);
    });

    // Type filter
    document.getElementById('typeFilter').addEventListener('change', function() {
        filterByType(this.value);
    });

    // Search by ID
    document.getElementById('searchById').addEventListener('click', function() {
        const roadId = document.getElementById('idSearch').value.trim();
        if (roadId) {
            searchById(roadId);
        } else {
            alert('Please enter a road ID');
        }
    });

    // Search on Enter key
    document.getElementById('idSearch').addEventListener('keypress', function(e) {
        if (e.key === 'Enter') {
            const roadId = this.value.trim();
            if (roadId) {
                searchById(roadId);
            }
        }
    });

    // Clear search
    document.getElementById('clearSearch').addEventListener('click', function() {
        clearSearch();
        document.getElementById('typeFilter').value = '';
        currentFilter = '';
        renderMap();
        updateStats();
    });

    // Reset view button
    document.getElementById('resetView').addEventListener('click', resetView);

    // Fit bounds button
    document.getElementById('fitBounds').addEventListener('click', fitBounds);

    // Zoom buttons
    document.getElementById('zoomIn').addEventListener('click', () => {
        svg.transition()
            .call(d3.zoom().scaleBy, 1.5)
            .on('end', () => {
                const transform = d3.zoomTransform(svg.node());
                updateTiles(transform);
            });
    });

    document.getElementById('zoomOut').addEventListener('click', () => {
        svg.transition()
            .call(d3.zoom().scaleBy, 1/1.5)
            .on('end', () => {
                const transform = d3.zoomTransform(svg.node());
                updateTiles(transform);
            });
    });

    // Toggle labels (placeholder function)
    document.getElementById('toggleLabels').addEventListener('click', () => {
        alert('Label feature to be implemented');
    });
}

// Adjust on window resize
window.addEventListener('resize', () => {
    width = document.getElementById('map-container').clientWidth;
    height = document.getElementById('map-container').clientHeight;
    svg.attr('width', width).attr('height', height);
    projection.translate([width / 2, height / 2]);
    if (geojsonData) {
        const transform = d3.zoomTransform(svg.node());
        updateTiles(transform);
        renderMap();
    }
});

// Initialize
init();
