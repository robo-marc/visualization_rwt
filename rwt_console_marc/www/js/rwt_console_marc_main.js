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
    NUMBER: '',
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
  var sortCol;
  var isAsc = false;

  // for grid
  var columns = [
    { id: '#', name: COLUMN_NAMES.DISPLAY_NUMBER, field: '#', width: 80, sortable: true, formatter: highlightFormatter },
    { id: 'Message', name: COLUMN_NAMES.MESSAGE, field: 'Message', width: 300, sortable: true, formatter: highlightFormatter },
    { id: 'Severity', name: COLUMN_NAMES.SEVERITY, field: 'Severity', width: 70, sortable: true, formatter: severityFormatter },
    { id: 'Node', name: COLUMN_NAMES.NODE, field: 'Node', width: 100, sortable: true, formatter: highlightFormatter },
    { id: 'Stamp', name: COLUMN_NAMES.STAMP, field: 'Stamp', width: 100, sortable: true, formatter: highlightFormatter },
    { id: 'Topics', name: COLUMN_NAMES.TOPICS, field: 'Topics', width: 100, sortable: true, formatter: highlightFormatter },
    { id: 'Location', name: COLUMN_NAMES.LOCATION, field: 'Location', width: 100, sortable: true, formatter: highlightFormatter },
    { id: 'Number', name: COLUMN_NAMES.NUMBER, field: 'Number', width: -1, maxWidth: -1, minWidth: -1, resizable: false, headerCssClass: 'hidden', sortable: true }
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
    $(selectId).append(valueList.map(function (value) {
      return '<option value="' + value + '">' + value + '</option>';
    }).join('\n'));
    $(selectId).change();
  }

  // render grid
  function renderList() {
    if (isPaused) {
      return;
    }
    dataView.setItems(list);
    // grid.invalidate();

    $('#displaying-count').text(dataView.getLength());
    $('#message-count').text(list.length);

    //TODO: フィルタ条件の Node と Topic のドロップダウンに候補を追加する
  }

  // start subscribing
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
      var mes = msg.msg;
      var severityNumber = msg.level;
      var severityLabel = getSeverityLabel(msg.level);
      var node = msg.name;
      var stamp = formatTime(msg.header.stamp.secs, msg.header.stamp.nsecs);
      var milliSec = toMilliSec(msg.header.stamp.secs, msg.header.stamp.nsecs);
      var regular = ('0000000000' + msg.header.stamp.nsecs).slice(-9);
      var topics = msg.topics.join(',');
      var location = msg.file + ':' + msg.function + ':' + msg.line;

      var associationItem = {
        id: rowNumber,
        indent: 0,
        '#': '#' + rowNumber,
        Message: mes,
        SeverityNumber: severityNumber,
        Severity: severityLabel,
        Node: node,
        Stamp: stamp,
        MilliSec: milliSec,
        RawTime: msg.header.stamp.secs + '.' + regular,
        Topics: topics,
        Location: location,
        // Number: count
      };

      if (isAsc === false) {
        list.unshift(associationItem);
        if (list.length > maxCount) {
          list.pop();
        }
      } else {
        list.push(associationItem);
        if (list.length > maxCount) {
          list.shift();
        }
      }
      rowNumber++;
    });
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

    var arr = [];
    var itemList = _.cloneDeep(dataView.getItems().reverse());
    _.each(itemList, function (value, index) {
      delete value.id;
      delete value['#'];
      delete value.Stamp;
      delete value.MilliSec;
      // delete value.RawTime;
      delete value.indent;
      delete value.Severity;
      // delete value.Number;
      // delete value.SeverityNumber;
      arr.push(Object.keys(value).map(function (key) {
        return value[key];
      })
      );
    });
    var csvData = '';
    _.each(arr, function (value, index) {
      var row = value.join('";"');
      console.log(row);
      if (row) {
        csvData = '"' + row + '"' + '\n' + csvData;
      }
    });
    if (csvData !== '') {
      csvData = csvData.slice(0, -1) + '"' + '\n';
    }
    var csvText = 'message;severity;node;stamp;topics;location\n' + csvData;
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
    // console.log(file);
    if (!file) {
      return;
    }

    // should check extension?
    // if (!file.name.match('\.csv$')) {
    //   alert('.csv file only');
    //   return;
    // }

    $('#pause-button').click();

    // read text
    var reader = new FileReader();
    reader.readAsText(file);

    reader.onload = function () {
      var rows = reader.result.split('\n');
      var data = [];
      var tmp;
      for (var i = 0; i < rows.length; i++) {
        tmp = rows[i].substr(1, rows[i].length - 2); // remove heading and trailing double-quotation
        data[i] = tmp.split('";"'); // split and remove double-quotation
      }

      addDataToGrid(data);
    };
  });

  function addDataToGrid(dataList) {
    dataList = dataList.slice().reverse();

    // ignore header row and trailing empty row
    for (var i = 1; i < dataList.length - 1; i++) {
      if (dataList[i]) {
        var severityNumber = parseInt(dataList[i][1], 10);
        var severityLabel = getSeverityLabel(severityNumber);
        var time = dataList[i][3].split('.');
        var secs = time[0];
        var nsecs = time[1];
        var stamp = formatTime(secs, nsecs);
        var milliSec;
        try {
          milliSec = toMilliSec(parseInt(secs, 10), parseInt(nsecs, 10));
        } catch (e) {
          console.info(e);
        }
        var regular = ('0000000000' + nsecs).slice(-9);

        var associationItem = {
          id: rowNumber,
          indent: 0,
          '#': '#' + rowNumber,
          Message: dataList[i][0],
          SeverityNumber: severityNumber,
          Severity: severityLabel,
          Node: dataList[i][2],
          Stamp: stamp,
          MilliSec: milliSec,
          RawTime: secs + '.' + regular,
          Topics: dataList[i][4],
          Location: dataList[i][5],
          // Number: dataList[i][6]
        };
        // console.log(associationItem);

        if (isAsc === false) {
          list.unshift(associationItem);
          if (list.length > maxCount) {
            list.pop();
          }
        } else {
          list.push(associationItem);
          if (list.length > maxCount) {
            list.shift();
          }
        }
        rowNumber++;
      }
    }

    var view = list.slice().reverse();
    dataView.setItems(view);
    grid.invalidate();

    $('#displaying-count').text(dataView.getLength());
    $('#message-count').text(list.length);
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


  ////////////////////////////////////////
  // grid events

  // sort
  grid.onSort.subscribe(function (e, args) {
    sortCol = args.sortCol.field;
    isAsc = args.sortAsc;

    if (sortCol === '#') {
      sortCol = 'Stamp';
    }
    if (sortCol === 'Stamp') {
      dataView.sort(comparer, isAsc);
    } else if (sortCol !== 'Stamp') {
      // dataView.sort(comparer, isAsc);
      dataView.sort(comparer2, isAsc);
    }

    grid.invalidateAllRows();
    grid.render();
  });
  function comparer(a, b) {
    var x = a[sortCol], y = b[sortCol];
    return (x === y ? 0 : (x > y ? 1 : -1));
  }
  function comparer2(a, b) {
    var x = a[sortCol], y = b[sortCol];
    var z = a['Stamp'], w = b['Stamp'];
    return (x === y ? (z > w ? 1 : -1) : 0);
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
        // return alert('No groups to add.');
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
    tmpStr = tmpStr.substr(0, tmpStr.length - 1);

    var tmpList = tmpStr.split(',');

    // distinct
    var nodeList = tmpList.filter(function (item, idx, self) {
      return self.indexOf(item) === idx;
    });

    nodeList.sort();

    return nodeList;
  }

  function getTopicList(dataViewItems) {
    // item.Topics may be a comma separated string
    var tmpStr = '';
    _.each(dataViewItems, function (item, index) {
      tmpStr += item.Topics + ',';
    });
    tmpStr = tmpStr.substr(0, tmpStr.length - 1);

    var tmpList = tmpStr.split(',');

    // distinct
    var topicList = tmpList.filter(function (item, idx, self) {
      return self.indexOf(item) === idx;
    });

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