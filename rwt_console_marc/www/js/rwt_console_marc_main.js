$(function () {

  ////////////////////////////////////////
  // constants

  // for grid, dropdown
  var COLUMN_NAMES = {
    DISPLAY_NUMBER: '#',
    MESSAGE: 'Message',
    SEVERITY: 'Severity',
    NODE: 'Node',
    STAMP: 'Stamp',
    TOPICS: 'Topics',
    LOCATION: 'Location',
  };

  var FILTER_NAME_LIST = [
    COLUMN_NAMES.MESSAGE,
    COLUMN_NAMES.SEVERITY,
    COLUMN_NAMES.NODE,
    COLUMN_NAMES.STAMP,
    COLUMN_NAMES.TOPICS,
    COLUMN_NAMES.LOCATION,
  ];


  ////////////////////////////////////////
  // variables

  var ros = new ROSLIB.Ros();

  // config
  var maxCount = 20000;
  var refleshRate = 4;

  // status
  var isPaused = false;
  var rowNumber = 1;
  var sortCol = '#';
  var isAsc = false;

  // for grid
  var columns = [
    { id: '#', name: COLUMN_NAMES.DISPLAY_NUMBER, field: '#', width: 80, sortable: true, formatter: highlightFormatter },
    { id: 'Message', name: COLUMN_NAMES.MESSAGE, field: 'Message', width: 300, sortable: true, formatter: highlightFormatter },
    { id: 'Severity', name: COLUMN_NAMES.SEVERITY, field: 'Severity', width: 75, sortable: true, formatter: severityFormatter },
    { id: 'Node', name: COLUMN_NAMES.NODE, field: 'Node', width: 100, sortable: true, formatter: highlightFormatter },
    { id: 'Stamp', name: COLUMN_NAMES.STAMP, field: 'Stamp', width: 100, sortable: true, formatter: highlightFormatter },
    { id: 'Topics', name: COLUMN_NAMES.TOPICS, field: 'Topics', width: 100, sortable: true, formatter: highlightFormatter },
    { id: 'Location', name: COLUMN_NAMES.LOCATION, field: 'Location', width: 100, sortable: true, formatter: highlightFormatter },
  ];
  var options = {
    editable: true,
    enableAddRow: false,
    enableCellNavigation: true,
    asyncEditorLoading: false
  };
  var dataView = new Slick.Data.DataView({ inlineFilters: true });
  var grid = new Slick.Grid('#myGrid', dataView, columns, options);
  var list = [];

  // for filter
  var excludeFilter = [];
  var highlightFilter = [];
  var showOnlyHighlighted = false;


  ////////////////////////////////////////
  // functions

  function initScreen() {
    // common
    ros.autoConnect();

    // dialog hide
    $('#status-dialog').addClass('dialog_hidden');

    $('#pause-button').show();
    $('#resume-button').hide();
    $('#open_sub_button').click();

    // for dropdown
    setDropdownList('#exclude-select', FILTER_NAME_LIST);
    setDropdownList('#highlight-select', FILTER_NAME_LIST);

    // for grid
    dataView.beginUpdate();
    dataView.setItems([]);
    updateFilter();
    dataView.setFilter(myFilter);
    dataView.endUpdate();
    grid.setSortColumn(sortCol, isAsc);

    setCountLabelStatus();

    // for filter
    setAndConditionCanSelect('exclude');
    setAndConditionCanSelect('highlight');

    // start
    startSubscribing();
    setInterval(renderList, 1000 / refleshRate);
    setTimeout(adjustColumnWidth, 1000 / refleshRate); // initial adjust
  }

  function setDropdownList(selectId, valueList) {
    var $selectElm = $(selectId);
    var html = '';
    for (var i = 0, len = valueList.length; i < len; i++) {
      html += '<option value="' + valueList[i] + '">' + valueList[i] + '</option>';
    }
    $selectElm.empty();
    $selectElm.append(html);
    $selectElm.change();
  }

  function startSubscribing() {
    var sub = null;
    sub = new ROSLIB.Topic({
      ros: ros,
      name: '/rosout_agg',
      messageType: 'rosgraph_msgs/Log'
    });
    sub.subscribe(function (msg) {
      if (isPaused) {
        return;
      }
      var message = msg.msg;

      var severityNumber = msg.level;
      var severityLabel = getSeverityLabel(msg.level);

      var node = msg.name;

      var stamp = formatTime(msg.header.stamp.secs, msg.header.stamp.nsecs);
      var milliSec = toMilliSec(msg.header.stamp.secs, msg.header.stamp.nsecs);
      var regular = ('0000000000' + msg.header.stamp.nsecs).slice(-9);
      var rawTime = msg.header.stamp.secs + '.' + regular;

      var topics = msg.topics.join(',');

      var location = msg.file + ':' + msg.function + ':' + msg.line;

      var associationItem = {
        id: rowNumber,
        '#': '#' + rowNumber,
        Message: message,
        SeverityNumber: severityNumber,
        Severity: severityLabel,
        Node: node,
        Stamp: stamp,
        MilliSec: milliSec,
        RawTime: rawTime,
        Topics: topics,
        Location: location,
      };

      list.unshift(associationItem);
      if (list.length > maxCount) {
        list.pop();
      }
      rowNumber++;
    });
  }

  // render grid
  function renderList() {
    if (isPaused) {
      return;
    }

    dataView.setItems(list);
    sortDataView();
    // grid.invalidate();

    $('#displaying-count').text(dataView.getLength());
    $('#message-count').text(list.length);

    appendNodesToFilter();
    appendTopicsToFilter();
  }

  function appendNodesToFilter() {
    var $filterList = $('.node-value');
    if ($filterList.length <= 0) {
      return;
    }
    var listInGrid = getNodeList(list);
    appendOptionsToFilter($filterList, listInGrid);
  }

  function appendTopicsToFilter() {
    var $filterList = $('.topic-value');
    if ($filterList.length <= 0) {
      return;
    }
    var listInGrid = getTopicList(list);
    appendOptionsToFilter($filterList, listInGrid);
  }

  function appendOptionsToFilter($filterList, listInGrid) {
    $filterList.each(function (index, selectElm) {
      var $selectElm = $(selectElm);

      var selectedValue = $selectElm.val();
      var optValueList = listInGrid.slice(0, listInGrid.length); // copy array
      $selectElm.children().each(function (optIndex, optElm) {
        optValueList.push($(optElm).val());
      });
      optValueList = distinct(optValueList);

      var html = '';
      for (var i = 0, len = optValueList.length; i < len; i++) {
        html += '<option value="' + optValueList[i] + '">' + optValueList[i] + '</option>';
      }
      $selectElm.empty();
      $selectElm.append(html);
      $selectElm.val(selectedValue);
    });
  }

  function distinct(arr) {
    var result = [];
    var isFirst = true;
    var prevVal;
    var tmpArr = arr.slice(0, arr.length);
    tmpArr.sort();
    for (var i = 0, len = tmpArr.length; i < len; i++) {
      var val = tmpArr[i];
      if (prevVal !== val || isFirst) {
        result.push(val);
        if (isFirst) {
          isFirst = false;
        }
      }
      prevVal = val;
    }
    return result;
  }

  function highlightFormatter(row, cell, value, columnDef, dataContext) {
    if (value === null || value === undefined || dataContext === undefined) { return ''; }
    var matchedClass = '';
    if (dataContext['_matched'] === false) {
      matchedClass = 'notMatched';
    }
    return '<span class="' + matchedClass + '">' + value + '</span>';
  }

  function severityFormatter(row, cell, value, columnDef, dataContext) {
    if (value === null || value === undefined || dataContext === undefined) { return ''; }
    var matchedClass = '';
    if (dataContext['_matched'] === false) {
      matchedClass = 'notMatched';
    }

    return '<span class="severity ' + value + ' ' + matchedClass + '">' + value + '</span>';
  }

  function myFilter(item, args) {
    var isVisible;

    // excludeFilter
    var isMatchToExcludeFilter = FilterUtils.isMatchedToAnyFilterGroup(item, args.excludeFilter, false);
    isVisible = !isMatchToExcludeFilter;

    // highlightFilter
    var isMatchToHighlightFilter = FilterUtils.isMatchedToAnyFilterGroup(item, args.highlightFilter, true);
    if (args.showOnlyHighlighted && !isMatchToHighlightFilter) {
      isVisible = false;
    }

    return isVisible;
  }

  function updateFilter() {
    dataView.setFilterArgs({
      excludeFilter: excludeFilter,
      highlightFilter: highlightFilter,
      showOnlyHighlighted: showOnlyHighlighted,
    });
    dataView.refresh();
    grid.invalidateAllRows();
    grid.render();

    setCountLabelStatus();
  }

  function setCountLabelStatus() {
    var filterCount = $('#exclude .filter input.isEffective:checked').length
      + $('#highlight .filter input.isEffective:checked').length;
    if (filterCount > 0) {
      $('#displaying-count-area').removeClass('hidden');
    } else {
      $('#displaying-count-area').addClass('hidden');
    }
  }

  function getSeverityLabel(value) {
    var label;
    switch (value) {
      case 1:
        label = 'Debug';
        break;
      case 2:
        label = 'Info';
        break;
      case 4:
        label = 'Warn';
        break;
      case 8:
        label = 'Error';
        break;
      case 16:
        label = 'Fatal';
        break;
    }
    return label;
  }

  // format time
  function formatTime(secs, nsecs) {
    var intTime = secs;
    var d = new Date(intTime * 1000);
    var year = d.getFullYear();
    var month = d.getMonth() + 1;
    var day = d.getDate();
    var hour = ('0' + d.getHours()).slice(-2);
    var min = ('0' + d.getMinutes()).slice(-2);
    var sec = ('0' + d.getSeconds()).slice(-2);
    var regular = ('0000000000' + nsecs).slice(-9);
    var timestamp = hour + ':' + min + ':' + sec + '.' + regular + '(' + year + '-' + month + '-' + day + ')';
    return timestamp;
  }

  function toMilliSec(secs, nsecs) {
    return (secs || 0) * 1000 + (nsecs || 0) / 1000000.0;
  }


  ////////////////////////////////////////
  // button events

  // CSV save
  $('#save-button').on('click', function (e) {
    // do not preventDefault.

    var csvText = FILTER_NAME_LIST.join(';').toLowerCase();

    var displayCount = dataView.getLength();
    var lineList = [];
    for (var i = 0; i < displayCount; i++) {
      var item = dataView.getItem(i);

      var lineData = [];
      lineData.push(item['Message'].replace(/"/g, '\\"'));
      lineData.push(item['SeverityNumber'].toString());
      lineData.push(item['Node']);
      lineData.push(item['RawTime']); // nsecs already zero filled
      lineData.push(item['Topics']); // topics already joind with ','
      lineData.push(item['Location']);

      var lineText = '\"' + lineData.join('\";\"') + '\"';
      lineList.push(lineText);
    }
    csvText += '\n' + lineList.join('\n') + '\n';

    var blob = new Blob([csvText], { 'type': 'text/plain' });
    if (window.navigator.msSaveBlob) {
      window.navigator.msSaveBlob(blob, 'rwt_console_marc.csv');
      window.navigator.msSaveOrOpenBlob(blob, 'rwt_console_marc.csv');
    } else {
      $('#save-button').attr('href', window.URL.createObjectURL(blob));
    }
  });

  // CSV load
  $('#load-button').on('click', function (e) {
    e.preventDefault();

    $('#fileSelector').click();
  });
  $('#fileSelector').on('change', function (evt) {
    var file = evt.target.files[0];
    if (!file) {
      return;
    }

    $('#pause-button').click();
    var reader = new FileReader();
    reader.readAsText(file);

    reader.onload = function () {
      var lines = reader.result.split('\n');
      var hasPreFix;
      var hasSuffix;
      var lastWrapeed = false;
      var keepRows = [];
      var keepColumns = lines[0];

      if (lines.length < 2) {
        return;
      }

      for (var i = 1; i < lines.length; i++) {
        if (lines[i].length === 0) {
          continue;
        }
        if (lines[i] === '"') {
          hasPreFix = !lastWrapeed;
          hasSuffix = lastWrapeed;
        } else {
          hasPreFix = lines[i].charAt(0) === '"';
          hasSuffix = lines[i].charAt(lines[i].length - 1) === '"';
        }

        if (!hasPreFix && !lastWrapeed) {
          continue;
        }

        if (hasPreFix && lastWrapeed) {
          keepRows.pop();
        }

        if (lastWrapeed) {
          keepRows[keepRows.length - 1] = keepRows[keepRows.length - 1] + lines[i];
        } else {
          keepRows.push(lines[i]);
        }

        lastWrapeed = !hasSuffix;
      }

      keepRows.unshift(keepColumns);
      var rows = keepRows;
      var columns = rows[0].split(';');
      var colIndexs = {};
      for (var j = 0; j < columns.length; j++) {
        var str = columns[j];
        switch (str) {
          case 'message':
            colIndexs[COLUMN_NAMES.MESSAGE] = j;
            break;
          case 'severity':
            colIndexs[COLUMN_NAMES.SEVERITY] = j;
            break;
          case 'node':
            colIndexs[COLUMN_NAMES.NODE] = j;
            break;
          case 'stamp':
            colIndexs[COLUMN_NAMES.STAMP] = j;
            break;
          case 'topics':
            colIndexs[COLUMN_NAMES.TOPICS] = j;
            break;
          case 'location':
            colIndexs[COLUMN_NAMES.LOCATION] = j;
            break;
          default:
            console.info('Unknown column: %s', str);
            return;
        }
      }

      addDataToGrid(colIndexs, rows);
    };
  });

  function addDataToGrid(colIndexs, rows) {
    for (var i = 1, rowCount = rows.length; i < rowCount; i++) {
      var row = rows[i];
      if (!row) {
        continue;
      }
      row = row.substring(1, row.length - 1); // remove heading and trailing double-quotation
      var cells = row.split('";"'); // split and remove double-quotation

      var message = cells[colIndexs[COLUMN_NAMES.MESSAGE]].replace(/\\"/g, '"');

      var severityNumber = parseInt(cells[colIndexs[COLUMN_NAMES.SEVERITY]], 10);
      var severityLabel = getSeverityLabel(severityNumber);
      if (severityLabel === undefined) {
        console.info('Unknown severity value: %s', cells[colIndexs[COLUMN_NAMES.SEVERITY]]);
        continue;
      }

      var node = cells[colIndexs[COLUMN_NAMES.NODE]];

      var time = cells[colIndexs[COLUMN_NAMES.STAMP]].split('.');
      if (time.length !== 2) {
        console.info('Unknown timestamp format: %s', cells[colIndexs[COLUMN_NAMES.STAMP]]);
        continue;
      }
      var secs = time[0];
      var nsecs = time[1];
      var stamp = formatTime(secs, nsecs);
      var milliSec;
      try {
        milliSec = toMilliSec(parseInt(secs, 10), parseInt(nsecs, 10));
      } catch (e) {
        console.info(e);
        continue;
      }
      var regular = ('0000000000' + nsecs).slice(-9);
      var rawTime = secs + '.' + regular;

      var topics = cells[colIndexs[COLUMN_NAMES.TOPICS]]; // donot split by ','

      var location = cells[colIndexs[COLUMN_NAMES.LOCATION]];

      var associationItem = {
        id: rowNumber,
        '#': '#' + rowNumber,
        Message: message,
        SeverityNumber: severityNumber,
        Severity: severityLabel,
        Node: node,
        Stamp: stamp,
        MilliSec: milliSec,
        RawTime: rawTime,
        Topics: topics,
        Location: location,
      };

      list.unshift(associationItem);
      if (list.length > maxCount) {
        list.pop();
      }
      rowNumber++;
    }

    dataView.setItems(list);
    sortDataView();
    grid.invalidate();

    $('#displaying-count').text(dataView.getLength());
    $('#message-count').text(list.length);

    appendNodesToFilter();
    appendTopicsToFilter();
  }

  // dialog display
  function showDialog(dataViewItem) {
    $('#dialog_header_title').text('Message Viewer');
    $('#dialog_node').text(dataViewItem.Node);
    $('#dialog_time').text(dataViewItem.Stamp);
    $('#dialog_severity').text(dataViewItem.Severity);
    $('#dialog_topics').text(dataViewItem.Topics);
    $('#dialog_message').text(dataViewItem.Message);
    $('#dialog_location').text(dataViewItem.Location);

    $('#status-dialog').removeClass('dialog_hidden');
  }

  // pause
  $('#pause-button').on('click', function (e) {
    e.preventDefault();
    isPaused = true;
    $('#pause-button').hide();
    $('#resume-button').show();
  });

  // resume
  $('#resume-button').on('click', function (e) {
    e.preventDefault();
    isPaused = false;
    $('#pause-button').show();
    $('#resume-button').hide();
  });

  // clear list
  $('#clear-button').on('click', function (e) {
    e.preventDefault();

    list.length = 0; // clear list
    rowNumber = 1;
    dataView.setItems(list);
    grid.invalidate();
    $('#displaying-count').text('0');
    $('#message-count').text('0');
  });

  $('#open_sub_button').on('click', function (e) {
    e.preventDefault();
    $('#contents_sub').slideDown('normal');
    $('#close_sub_button').show();
    $('#open_sub_button').hide();
  });

  $('#close_sub_button').on('click', function (e) {
    e.preventDefault();
    $('#contents_sub').slideUp('normal');
    $('#close_sub_button').hide();
    $('#open_sub_button').show();
  });

  // dialog close
  $('#close').on('click', function () {
    $('#status-dialog').addClass('dialog_hidden');
  });

  ////////////////////////////////////////
  // grid events

  //dialog
  grid.onDblClick.subscribe(function (e, args) {
    var item = dataView.getItem(args.row);
    showDialog(item);
  });

  // sort
  grid.onSort.subscribe(function (e, args) {
    sortCol = args.sortCol.field;
    isAsc = args.sortAsc;

    if (sortCol === '#') {
      isAsc = false;
      args.sortAsc = false;
      grid.setSortColumn(sortCol, isAsc);
    }

    sortDataView();

    grid.invalidateAllRows();
    grid.render();
  });

  function sortDataView() {
    var func;
    switch (sortCol) {
      case '#':
        func = comparerNumber;
        break;
      case 'Severity':
        func = comparerSeverity;
        break;
      case 'Stamp':
        func = comparerStamp;
        break;
      default:
        func = comparerCommon;
        break;
    }
    dataView.sort(func, isAsc);
  }

  function comparerNumber(a, b) {
    var x = a['id'];
    var y = b['id'];
    if (x < y) {
      return -1;
    } else if (x > y) {
      return 1;
    }
    return comparerStamp(a, b);
  }
  function comparerSeverity(a, b) {
    var x = a['SeverityNumber'];
    var y = b['SeverityNumber'];
    if (x < y) {
      return -1;
    } else if (x > y) {
      return 1;
    }
    return comparerStamp(a, b);
  }
  function comparerStamp(a, b) {
    var x = a['MilliSec'];
    var y = b['MilliSec'];
    if (x < y) {
      return -1;
    } else if (x > y) {
      return 1;
    }
    return 0;
  }
  function comparerCommon(a, b) {
    var x = a[sortCol], y = b[sortCol];
    if (x < y) {
      return -1;
    } else if (x > y) {
      return 1;
    }
    return 0;
  }

  dataView.onRowCountChanged.subscribe(function (e, args) {
    grid.updateRowCount();
    grid.render();
  });

  dataView.onRowsChanged.subscribe(function (e, args) {
    grid.invalidateRows(args.rows);
    grid.render();
  });

  $(window).on('load resize', function () {
    adjustColumnWidth();
  });

  function adjustColumnWidth() {
    grid.resizeCanvas();
    grid.autosizeColumns();
  }


  ////////////////////////////////////////
  // add filter

  $('#add-exclude-button').on('click', function () {
    addFilter('exclude');
  });

  $('#add-highlight-button').on('click', function () {
    addFilter('highlight');
  });

  function addFilter(parentId) {
    // validate input
    var conditionType = $('input[name="' + parentId + '_input"]:checked').val();
    if (conditionType === 'AND') {
      if ($('#' + parentId + ' > li').length === 0) {
        console.info('No groups to add.');
        return;
      }
    }

    // build html
    var html;
    var filterName = $('#' + parentId + '-select').val();
    switch (filterName) {
      case COLUMN_NAMES.MESSAGE:
        html = FilterUtils.getMessageFilterHtml(conditionType);
        break;
      case COLUMN_NAMES.SEVERITY:
        html = FilterUtils.getSeverityFilterHtml(conditionType);
        break;
      case COLUMN_NAMES.NODE:
        var nodeList = getNodeList(list);
        html = FilterUtils.getNodesFilterHtml(conditionType, nodeList);
        break;
      case COLUMN_NAMES.STAMP:
        html = FilterUtils.getStampFilterHtml(conditionType);
        break;
      case COLUMN_NAMES.TOPICS:
        var topicList = getTopicList(list);
        html = FilterUtils.getTopicsFilterHtml(conditionType, topicList);
        break;
      case COLUMN_NAMES.LOCATION:
        html = FilterUtils.getLocationFilterHtml(conditionType);
        break;
      default:
        return;
    }

    // insert below active item, or append to last
    var $activeElm = $('#' + parentId + ' > .active');
    if ($activeElm.length > 0) {
      $activeElm.after(html);
    } else {
      $('#' + parentId).append(html);
    }

    setAndConditionCanSelect(parentId);
    setFirstItemCanDelete(parentId);

    $('#' + parentId).trigger('change');
  }

  function getNodeList(dataViewItems) {
    // item.Node may be a comma separated string
    var tmpStr = '';
    _.each(dataViewItems, function (item, index) {
      tmpStr += item.Node + ',';
    });
    if (tmpStr === '') {
      return [];
    }
    tmpStr = tmpStr.substr(0, tmpStr.length - 1);

    var nodeList = tmpStr.split(',');
    nodeList = distinct(nodeList);
    nodeList.sort();

    return nodeList;
  }

  function getTopicList(dataViewItems) {
    // item.Topics may be a comma separated string
    var tmpStr = '';
    _.each(dataViewItems, function (item, index) {
      tmpStr += item.Topics + ',';
    });
    if (tmpStr === '') {
      return [];
    }
    tmpStr = tmpStr.substr(0, tmpStr.length - 1);

    var topicList = tmpStr.split(',');
    topicList = distinct(topicList);
    topicList.sort();

    return topicList;
  }

  function setAndConditionCanSelect(parentId) {
    var canSelect = true;

    var items = $('#' + parentId + ' > .filter');
    if (items.length <= 0) {
      canSelect = false;
    }

    var $orBtn = $('#' + parentId + '_or');
    var $andBtn = $('#' + parentId + '_and');
    var $andLabel = $('#' + parentId + '_and_label');
    if (canSelect) {
      $andBtn.prop('disabled', false);
      $andLabel.removeClass('disabled');
    } else {
      $orBtn.prop('checked', true);
      $andBtn.prop('disabled', true);
      $andLabel.addClass('disabled');
    }
  }

  function setFirstItemCanDelete(parentId) {
    var canDelete = true;

    var items = $('#' + parentId + ' > .filter');
    if (items.length >= 2) {
      var $andElm = $('#' + parentId + ' > li.filter:nth-child(2) .label.and');
      if ($andElm.length !== 0) {
        canDelete = false;
      }
    }

    var $deleteBtn = $('#' + parentId + ' > li.filter:first-child .delete-button');
    if (canDelete) {
      $deleteBtn.show();
    } else {
      $deleteBtn.hide();
    }
  }


  ////////////////////////////////////////
  // delete filter

  $('#exclude').on('click', '.delete-button', function (e) {
    e.preventDefault();
    deleteFilter('exclude', this);
  });

  $('#highlight').on('click', '.delete-button', function (e) {
    e.preventDefault();
    deleteFilter('highlight', this);
  });

  function deleteFilter(parentId, selectedItem) {
    $(selectedItem).parentsUntil('.filter').parent().remove();
    setAndConditionCanSelect(parentId);
    setFirstItemCanDelete(parentId);
    $('#' + parentId).trigger('change');
  }


  ////////////////////////////////////////
  // set active filter

  $('#exclude').on('click', '.filter', function () {
    setActiveFilterItem('exclude', this);
  });

  $('#highlight').on('click', '.filter', function () {
    setActiveFilterItem('highlight', this);
  });

  function setActiveFilterItem(parentId, selectedItem) {
    var $elm = $(selectedItem);
    if ($elm.hasClass('active')) {
      return;
    }
    $('#' + parentId).children().each(function (index, element) {
      $(element).removeClass('active');
    });
    $elm.addClass('active');
  }


  ////////////////////////////////////////
  // filter input

  // show-only-highlighted
  $('#show-only-highlighted').on('click', function () {
    showOnlyHighlighted = $(this).prop('checked');
    updateFilter();
  });

  // toggle severity on/off
  $('#exclude').on('click', '.severities-select-button', function (e) {
    e.preventDefault();
    $(this).toggleClass('active');
    $('#exclude').trigger('change');
  });

  $('#highlight').on('click', '.severities-select-button', function (e) {
    e.preventDefault();
    $(this).toggleClass('active');
    $('#highlight').trigger('change');
  });

  // stamp checkbox on/off
  $('#exclude').on('change', '.use-end-stamp', function () {
    setUseEndStampState(this);
  });

  $('#highlight').on('change', '.use-end-stamp', function () {
    setUseEndStampState(this);
  });

  function setUseEndStampState(selectedItem) {
    var $elm = $(selectedItem);
    var isChecked = $elm.prop('checked');
    $elm.next().find('input.end').each(function (index, element) {
      $(element).prop('disabled', !isChecked);
    });
  }


  ////////////////////////////////////////
  // when filter changed

  $('#exclude').on('change', function () {
    filterOnChangeHandler('exclude');
  });

  $('#highlight').on('change', function () {
    filterOnChangeHandler('highlight');
  });

  function filterOnChangeHandler(parentId) {
    var $filterList = $('#' + parentId + ' li.filter');

    var groupList = [];
    var valueList = [];

    $filterList.each(function (index, nativeElm) {
      var $elm = $(nativeElm);

      var isNewGroup = ($elm.find('span.label.and').length === 0);
      if (isNewGroup) {
        valueList = [];
        groupList.push(valueList);
      }

      var filterValue = getFilterValue($elm);

      valueList.push(filterValue);
    });

    if (parentId === 'exclude') {
      excludeFilter = groupList;
    } else {
      highlightFilter = groupList;
    }

    updateFilter();
  }

  function getFilterValue($elm) {
    var isEnabled = $elm.find('input.isEffective').prop('checked');
    var text;
    var re;

    if ($elm.hasClass(FilterUtils.FILTER_TYPE.MESSAGE)) {
      text = $elm.find('.message-value').val();
      re = undefined;
      if ($elm.find('.regex').prop('checked')) {
        try {
          re = new RegExp(text);
        } catch (e) {
          console.info(e);
        }
      }
      return {
        filterType: FilterUtils.FILTER_TYPE.MESSAGE,
        isEnabled: isEnabled,
        value: {
          message: text,
          regex: re,
        }
      };

    } else if ($elm.hasClass(FilterUtils.FILTER_TYPE.SEVERITY)) {
      return {
        filterType: FilterUtils.FILTER_TYPE.SEVERITY,
        isEnabled: isEnabled,
        value: getSeveritiesValue($elm)
      };

    } else if ($elm.hasClass(FilterUtils.FILTER_TYPE.NODE)) {
      return {
        filterType: FilterUtils.FILTER_TYPE.NODE,
        isEnabled: isEnabled,
        value: $elm.find('.node-value').val()
      };

    } else if ($elm.hasClass(FilterUtils.FILTER_TYPE.STAMP)) {
      var beginDateStr = $elm.find('.date.begin').val() + ' ' + $elm.find('.time.begin').val();
      var endDateStr = $elm.find('.date.end').val() + ' ' + $elm.find('.time.end').val();
      var beginDate;
      var endDate;
      try {
        beginDate = Date.parse(beginDateStr);
      } catch (e) {
        console.info(e);
      }
      try {
        endDate = Date.parse(endDateStr);
      } catch (e) {
        console.info(e);
      }
      return {
        filterType: FilterUtils.FILTER_TYPE.STAMP,
        isEnabled: isEnabled,
        value: {
          useEndTime: $elm.find('.use-end-stamp').prop('checked'),
          beginDate: beginDate,
          endDate: endDate,
        }
      };

    } else if ($elm.hasClass(FilterUtils.FILTER_TYPE.TOPICS)) {
      return {
        filterType: FilterUtils.FILTER_TYPE.TOPICS,
        isEnabled: isEnabled,
        value: $elm.find('.topic-value').val()
      };

    } else if ($elm.hasClass(FilterUtils.FILTER_TYPE.LOCATION)) {
      text = $elm.find('.location-value').val();
      re = undefined;
      if ($elm.find('.regex').prop('checked')) {
        try {
          re = new RegExp(text);
        } catch (e) {
          console.info(e);
        }
      }
      return {
        filterType: FilterUtils.FILTER_TYPE.LOCATION,
        isEnabled: isEnabled,
        value: {
          location: text,
          regex: re,
        }
      };

    }
  }

  function getSeveritiesValue($elm) {
    var severity = 0;
    $elm.find('.severities-select-button.active').each(function (index, item) {
      var value = parseInt($(item).data('value'), 10);
      if (!isNaN(value)) {
        severity |= value;
      }
    });
    return severity;
  }


  ////////////////////////////////////////
  // start screen

  initScreen();
});
