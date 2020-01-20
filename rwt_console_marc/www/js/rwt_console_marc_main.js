function requiredFieldValidator(value) {
  if (value === null || value === undefined || !value.length) {
    return { valid: false, msg: 'This is a required field' };
  } else {
    return { valid: true, msg: null };
  }
}

$(function () {

  ////////////////////////////////////////
  // constants

  // for grid, dropdown
  var FILTER_NAME = {
    MESSAGE: 'Message',
    SEVERITY: 'Severity',
    NODE: 'Node',
    STAMP: 'Stamp',
    TOPICS: 'Topics',
    LOCATION: 'Location',
    NUMBER: 'Number',
  };

  var FILTER_NAME_LIST = [
    FILTER_NAME.MESSAGE,
    FILTER_NAME.SEVERITY,
    FILTER_NAME.NODE,
    FILTER_NAME.STAMP,
    FILTER_NAME.TOPICS,
    FILTER_NAME.LOCATION,
    // NUMBER is hidden field
  ];

  var jQueryEachBreakValue = false;
  var jQueryEachContinueValue = true;



  ////////////////////////////////////////
  // variables

  var ros = new ROSLIB.Ros();

  // config
  var maxCount = 20000;

  // status
  var isPaused = false;
  var rowNumber = 1;
  var sortCol;
  var isAsc = false;

  // for grid
  var columns = [
    { id: '#', name: '#', field: '#', width: 80, sortable: true },
    { id: 'Message', name: FILTER_NAME.MESSAGE, field: 'Message', width: 300, sortable: true },
    { id: 'Severity', name: FILTER_NAME.SEVERITY, field: 'Severity', width: 70, sortable: true, formatter: severityFormatter },
    { id: 'Node', name: FILTER_NAME.NODE, field: 'Node', width: 100, sortable: true },
    { id: 'Stamp', name: FILTER_NAME.STAMP, field: 'Stamp', width: 100, sortable: true },
    { id: 'Topics', name: FILTER_NAME.TOPICS, field: 'Topics', width: 100, sortable: true },
    { id: 'Location', name: FILTER_NAME.LOCATION, field: 'Location', width: 100, sortable: true },
    { id: 'Number', name: FILTER_NAME.NUMBER, field: 'Number', width: -1, maxWidth: -1, minWidth: -1, resizable: false, headerCssClass: 'hidden', sortable: true }
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
  var filterScript = 'function init() {return true; } init();';
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

    // for filter
    setAndConditionCanSelect('exclude');
    setAndConditionCanSelect('highlight');

    // start
    startSubscribing();
    setInterval(renderList, 500);
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
      var severity = getErrorLevel(msg.level);
      var node = msg.name;
      var time = formatTime(msg.header.stamp.secs, msg.header.stamp.nsecs);
      var regular = ('0000000000' + msg.header.stamp.nsecs).slice(-9);
      var stamp = time;
      var topics = msg.topics.join(',');
      var location = msg.file + ':' + msg.function + ':' + msg.line;

      var associationItem = {
        id: rowNumber,
        indent: 0,
        '#': '#' + rowNumber,
        Message: mes,
        SeverityNumber: severityNumber,
        Severity: severity,
        Node: node,
        Stamp: stamp,
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

      $('#message-number').text(list.length);
    });
  }

  function severityFormatter(row, cell, value, columnDef, dataContext) {
    // console.log(dataContext);
    if (value === null || value === undefined || dataContext === undefined) { return ''; }
    return '<span class="severity ' + value + '">' + value + '</span>';
  }

  function isItemMatchFilter(item, filterList) {
    // var filterList = args.excludeFilter;
    var isExAnyGrpMatch = false;
    for (var exGrpIdx = 0, exGrpLen = filterList.length; exGrpIdx < exGrpLen; exGrpIdx++) {
      var exGrp = filterList[exGrpIdx]; // one filter group
      var isExGrpMatch = true;

      for (var exFltIdx = 0, exFltLen = exGrp.length; exFltIdx < exFltLen; exFltIdx++) {
        var exFlt = exGrp[exFltIdx]; // one filter
        // console.log(exFlt);

        if (!exFlt.isEnabled) {
          console.log('isExGrpMatch:' + (isExGrpMatch ? 'true' : 'false'));
          continue;
        }

        var exFltVal = exFlt.value;
        switch (exFlt.filterType) {
          case FilterUtils.FILTER_TYPE.MESSAGE:
            if (exFltVal.message + '' === '') {
              isExGrpMatch = false;
              console.log('isExGrpMatch:' + (isExGrpMatch ? 'true' : 'false'));
              continue;
            }
            if (exFltVal.isRegex) {

            } else {
              isExGrpMatch &= item.Message.indexOf(exFltVal.message) !== -1;
              // console.log('item.Message:' + item.Message);
              // console.log('exFltVal.message:' + exFltVal.message);
              // console.log('isMatch:' + (item.Message.indexOf(exFltVal.message) !== -1));
            }
            break;
        }

      }
      console.log('isExGrpMatch result:' + (isExGrpMatch ? 'true' : 'false'));
      isExAnyGrpMatch |= isExGrpMatch;
    }
    console.log('isExAnyGrpMatch result:' + (isExAnyGrpMatch ? 'true' : 'false'));
    return isExAnyGrpMatch;
  }

  function myFilter(item, args) {
    // console.log(args.excludeFilter);
    // console.log(args.highlightFilter);
    // console.log(args.showOnlyHighlighted);
    var isVisible = true;

    // excludeFilter
    var isMatchToExcludeFilter = FilterUtils.isMatchToFilter(item, args.excludeFilter);
    // var excludeFilter = args.excludeFilter;
    // var isExAnyGrpMatch = false;
    // for (var exGrpIdx = 0, exGrpLen = excludeFilter.length; exGrpIdx < exGrpLen; exGrpIdx++) {
    //   var exGrp = excludeFilter[exGrpIdx]; // one filter group
    //   var isExGrpMatch = true;

    //   for (var exFltIdx = 0, exFltLen = exGrp.length; exFltIdx < exFltLen; exFltIdx++) {
    //     var exFlt = exGrp[exFltIdx]; // one filter
    //     // console.log(exFlt);

    //     if (!exFlt.isEnabled) {
    //       console.log('isExGrpMatch:' + (isExGrpMatch ? 'true' : 'false'));
    //       continue;
    //     }

    //     var exFltVal = exFlt.value;
    //     switch (exFlt.filterType) {
    //       case FilterUtils.FILTER_TYPE.MESSAGE:
    //         if (exFltVal.message + '' === '') {
    //           isExGrpMatch = false;
    //           console.log('isExGrpMatch:' + (isExGrpMatch ? 'true' : 'false'));
    //           continue;
    //         }
    //         if (exFltVal.isRegex) {

    //         } else {
    //           isExGrpMatch &= item.Message.indexOf(exFltVal.message) !== -1;
    //           // console.log('item.Message:' + item.Message);
    //           // console.log('exFltVal.message:' + exFltVal.message);
    //           // console.log('isMatch:' + (item.Message.indexOf(exFltVal.message) !== -1));
    //         }
    //         break;
    //     }

    //   }
    //   console.log('isExGrpMatch result:' + (isExGrpMatch ? 'true' : 'false'));
    //   isExAnyGrpMatch |= isExGrpMatch;
    // }
    // console.log('isExAnyGrpMatch result:' + (isExAnyGrpMatch ? 'true' : 'false'));
    isVisible = !isMatchToExcludeFilter;

    item['_highlited_test'] = true;

    return isVisible;
    // return true; // for development(no filter)
  }

  function updateFilter() {
    // console.log('filter update');
    dataView.setFilterArgs({
      excludeFilter: excludeFilter,
      highlightFilter: highlightFilter,
      showOnlyHighlighted: showOnlyHighlighted,
    });
    dataView.refresh();
  }

  // get error level
  function getErrorLevel(level) {
    var severity;
    switch (level) {
      case 1:
        severity = 'Debug';
        break;
      case 2:
        severity = 'Info';
        break;
      case 4:
        severity = 'Warn';
        break;
      case 8:
        severity = 'Error';
        break;
      case 16:
        severity = 'Fatal';
        break;
    }
    return severity;
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


  ////////////////////////////////////////
  // button events

  // CSV download
  $('#download-button').on('click', function (e) {
    // do not preventDefault.

    var arr = [];
    var itemList = _.cloneDeep(dataView.getItems().reverse());
    _.each(itemList, function (value, index) {
      delete value.id;
      delete value['#'];
      delete value.Stamp;
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
      $('#download-button').attr('href', window.URL.createObjectURL(blob));
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
        var time = dataList[i][3].split('.');
        var secs = time[0];
        var nsecs = time[1];

        var severityNumber = parseInt(dataList[i][1], 10);
        var associationItem = {
          id: rowNumber,
          indent: 0,
          '#': '#' + rowNumber,
          Message: dataList[i][0],
          SeverityNumber: severityNumber,
          Severity: getErrorLevel(severityNumber),
          Node: dataList[i][2],
          Stamp: formatTime(secs, nsecs),
          // RawTime: msg.header.stamp.secs + '.' + regular,
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

    $('#message-number').text(list.length);
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
    $('#message-number').text('0');
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
      dataView.sort(comparer, isAsc);
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
    grid.resizeCanvas();
    grid.autosizeColumns();
  });

  dataView.onRowsChanged.subscribe(function (e, args) {
    grid.invalidateRows(args.rows);
    grid.render();
  });

  $(window).on('load resize', function () {
    grid.resizeCanvas();
    grid.autosizeColumns();

    // prevent the delete button from being hidden when switching screens vertically
    grid.resizeCanvas();
  });


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
      case FILTER_NAME.MESSAGE:
        html = FilterUtils.getMessageFilterHtml(conditionType);
        break;
      case FILTER_NAME.SEVERITY:
        html = FilterUtils.getSeverityFilterHtml(conditionType);
        break;
      case FILTER_NAME.NODE:
        var nodeList = getNodeList(list);
        html = FilterUtils.getNodesFilterHtml(conditionType, nodeList);
        break;
      case FILTER_NAME.STAMP:
        html = FilterUtils.getStampFilterHtml(conditionType);
        break;
      case FILTER_NAME.TOPICS:
        var topicList = getTopicList(list);
        html = FilterUtils.getTopicsFilterHtml(conditionType, topicList);
        break;
      case FILTER_NAME.LOCATION:
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

  // function getElementName(element) {
  //   var elementName = element.attr('class');
  //   switch (elementName) {
  //     case 'message':
  //       elementName = 'Message';
  //       break;

  //     case 'severities':
  //       elementName = 'Severity';
  //       break;

  //     case 'node':
  //       elementName = 'Node';
  //       break;

  //     case 'time':
  //       elementName = 'Stamp';
  //       break;

  //     case 'topics':
  //       elementName = 'Topics';
  //       break;

  //     case 'location':
  //       elementName = 'Location';
  //       break;
  //   }
  //   return elementName;
  // }

  // function getFilterValue(elementName, $elm) {
  //   var filterValue;
  //   switch (elementName) {
  //     case 'Message':
  //       filterValue = $elm.find('.message-value').val();
  //       break;

  //     case 'Severity':
  //       filterValue = $elm.find('.severities-select-button.active');
  //       console.log(filterValue);
  //       break;

  //     case 'Node':
  //       filterValue = $elm.find('.node-value').val();
  //       break;

  //     case 'Stamp':
  //       filterValue = $elm.find('.topic-value').val();
  //       break;

  //     case 'Topics':
  //       filterValue = $elm.find('.topic-value').val();
  //       break;

  //     case 'Location':
  //       filterValue = $elm.find('.location-value').val();
  //       break;
  //   }
  //   return filterValue;
  // }

  function getSeveritiesValue($elm) {
    var severity = 0;
    $elm.find('.severities-select-button.active').each(function (index, item) {
      var value = parseInt($(item).data('value'), 10);
      if (!isNaN(value)) {
        severity += value;
      }
    });
    return severity;
  }

  function getFilterValue($elm) {
    var isEnabled = $elm.find('input.isEffective').prop('checked');

    if ($elm.hasClass(FilterUtils.FILTER_TYPE.MESSAGE)) {
      return {
        filterType: FilterUtils.FILTER_TYPE.MESSAGE,
        isEnabled: isEnabled,
        value: {
          message: $elm.find('.message-value').val(),
          isRegex: $elm.find('.regex').prop('checked'),
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
      return {
        filterType: FilterUtils.FILTER_TYPE.STAMP,
        isEnabled: isEnabled,
        value: {
          useEndTime: $elm.find('.use-end-stamp').prop('checked'),
          beginTime: $elm.find('.time.begin').val(),
          beginDate: $elm.find('.date.begin').val(),
          endTime: $elm.find('.time.end').val(),
          endDate: $elm.find('.date.end').val(),
        }
      };

    } else if ($elm.hasClass(FilterUtils.FILTER_TYPE.TOPICS)) {
      return {
        filterType: FilterUtils.FILTER_TYPE.TOPICS,
        isEnabled: isEnabled,
        value: $elm.find('.topic-value').val()
      };

    } else if ($elm.hasClass(FilterUtils.FILTER_TYPE.LOCATION)) {
      return {
        filterType: FilterUtils.FILTER_TYPE.NODE,
        isEnabled: isEnabled,
        value: {
          location: $elm.find('.location-value').val(),
          isRegex: $elm.find('.regex').prop('checked'),
        }
      };

    }
  }

  function filterOnChange(parentId) {

    var script = '';
    var $filterList = $('#' + parentId + ' li.filter');
    var isParentEffective = true;
    var isParent = false;
    var isTopic = false;
    var isSeverity = false;
    var headScript = 'function filter(item){';
    var $checkbox;
    var elementName;
    // var filterValue;

    var groupList = [];
    var valueList = [];

    // var isSelectType = true;
    var filterCount = $filterList.length;
    $filterList.each(function (index, nativeElm) {
      console.log('-----');
      var $elm = $(nativeElm);

      var isNewGroup = ($elm.find('span.label.and').length === 0);
      // console.log('isNewGroup:' + isNewGroup);
      if (isNewGroup) {
        valueList = [];
        groupList.push(valueList);
      }

      var filterValue = getFilterValue($elm);
      // console.log(filterValue);
      valueList.push(filterValue);

      // if (isNewGroup) {
      //   // isParent = false;

      //   //OR時の処理
      //   $checkbox = $($elm.find('input.isEffective'));
      //   isParentEffective = $checkbox.prop('checked');

      //   if (!isParentEffective) {
      //     console.log('filter disabled');
      //     return jQueryEachContinueValue;
      //   }
      //   console.log('filter enabled');

      //   if (isParentEffective) {
      //     if (index !== 0 && isParent) {
      //       script = script + '){ return false; }';
      //     }
      //     // console.log('parent in!!!!');

      //     isParent = true;
      //     //選択系
      //     //node
      //     elementName = getElementName($elm);
      //     // filterValue = getFilterValue(elementName, $elm);
      //     filterValue = getFilterValue($elm);
      //     // console.log(elementName);
      //     // console.log(filterValue);

      //     switch (elementName) {
      //       case 'Message':
      //         script = script + ' if(' + 'item["Message"]' + '.indexOf("' + filterValue + '") !== -1';
      //         break;

      //       case 'Severity':
      //         if (!isSeverity) {
      //           isSeverity = true;
      //           headScript = headScript + ' var severity=[];';

      //           // for (var key in filterValue) {
      //           _.each(filterValue, function (value, index) {
      //             console.log($(value).data());
      //             var severityValue = $(value).data();
      //             console.log(severityValue['value']);
      //             headScript = headScript + ' severity.push("' + severityValue['value'] + '"); ';
      //           });
      //         }
      //         script = script + ' if( severity.indexOf(' + 'item["Severity"]' + ') !== -1';
      //         break;

      //       case 'Node':
      //         script = script + ' if(' + 'item["Node"]' + '.indexOf("' + filterValue + '") !== -1';
      //         break;

      //       case 'Stamp':
      //         break;

      //       case 'Topic':
      //         if (!isTopic) {
      //           isTopic = true;
      //           headScript = headScript + ' var topicName = item["Topics"].split(",");';
      //         }
      //         script = script + ' if(' + 'topicName' + '.indexOf("' + filterValue + '") !== -1';
      //         break;

      //       case 'Location':
      //         script = script + ' if(' + 'item["Location"]' + '.indexOf("' + filterValue + '") !== -1';
      //         break;
      //     }
      //   }

      // } else {
      //   if (!isParentEffective) {
      //     console.log('parent filter disabled');
      //     return jQueryEachContinue();
      //   }
      //   console.log('parent filter enabled');

      //   $checkbox = $($elm.find('input.isEffective'));
      //   var isChildEffective = $checkbox.prop('checked');
      //   if (!isChildEffective) {
      //     console.log('child filter disabled');
      //     return jQueryEachContinue();
      //   }
      //   console.log('child filter enabled');


      //   //AND時の処理
      //   elementName = getElementName($elm);
      //   // filterValue = getFilterValue(elementName, $elm);
      //   filterValue = getFilterValue($elm);

      //   if (isChildEffective) {

      //     switch (elementName) {
      //       case 'Message':
      //         script = script + ' &&' + 'item["' + elementName + '"]' + '.indexOf("' + filterValue + '") !== -1';
      //         break;

      //       case 'Severity':
      //         if (!isSeverity) {
      //           isSeverity = true;
      //           headScript =    updateFilter();
      // rValue, function (value, index) {
      //             console.log(value);
      //             var severityValue = $(value).data();
      //             headScript = headScript + ' severity.push("' + severityValue['value'] + '"); ';
      //           });
      //         }
      //         script = script + ' && severity.indexOf(' + 'item["' + elementName + '"]' + ') !== -1';
      //         break;

      //       case 'Node':
      //         script = script + ' &&' + 'item["' + elementName + '"]' + '.indexOf("' + filterValue + '") !== -1';
      //         break;

      //       case 'Stamp':

      //         break;

      //       case 'Topics':
      //         if (!isTopic) {
      //           isTopic = true;
      //           headScript = headScript + ' var topicName = item["Topics"].split(",");';
      //         }
      //         script = script + ' &&' + ' topicName' + '.indexOf("' + filterValue + '") !== -1';
      //         break;

      //       case 'Location':
      //         script = script + ' &&' + 'item["Location"]' + '.indexOf("' + filterValue + '") !== -1';
      //         break;
      //     }

      //   }
      // }

      // console.log('isParentEffective:' + isParentEffective);
    });
    // if (!isParent) {
    //   script = script + 'return true;} filter(item);';
    // } else {
    //   script = script + '){ return false; } return true ;} filter(item);';
    // }

    // console.log(groupList);
    if (parentId === 'exclude') {
      excludeFilter = groupList;
    } else {
      highlightFilter = groupList;
    }
    // filterScript = headScript + script;
    // console.log(filterScript);
    updateFilter();
  }

  //excludeフィルタの中の値に変化があった時の処理
  $('#exclude').on('change', function (e) {
    // e.preventDefault();
    filterOnChange('exclude');
  });

  $('#highlight').on('change', function (e) {
    // e.preventDefault();
    filterOnChange('highlight');
  });


  ////////////////////////////////////////
  // start screen

  initScreen();
});