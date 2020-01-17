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

  var FILTER_OPTION_LIST = [
    'Filter Selected',
    'Message',
    'Severity',
    'Node',
    'Stamp',
    'Topics',
    'Location',
  ];


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
    { id: 'Message', name: 'Message', field: 'Message', width: 300, sortable: true },
    { id: 'Severity', name: 'Severity', field: 'Severity', width: 70, sortable: true, formatter: severityFormatter },
    // { id: 'Severity', name: 'Severity', field: 'Severity', width: 70, sortable: true },
    { id: 'Node', name: 'Node', field: 'Node', width: 100, sortable: true },
    { id: 'Stamp', name: 'Stamp', field: 'Stamp', width: 100, sortable: true },
    { id: 'Topics', name: 'Topics', field: 'Topics', width: 100, sortable: true },
    { id: 'Location', name: 'Location', field: 'Location', width: 100, sortable: true },
    { id: 'Number', name: 'Number', field: 'Number', width: -1, maxWidth: -1, minWidth: -1, resizable: false, headerCssClass: 'hidden', sortable: true }
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


  ////////////////////////////////////////
  // functions

  function initScreen() {
    // common
    ros.autoConnect();

    $('#pause-button').show();
    $('#resume-button').hide();
    $('#open_sub_button').click();

    // for dropdown
    setDropdownList('#exclude-select', FILTER_OPTION_LIST);
    setDropdownList('#highlight-select', FILTER_OPTION_LIST);

    // for grid
    dataView.beginUpdate();
    dataView.setItems([]);
    dataView.setFilterArgs({
      filterScript: filterScript,
    });
    dataView.setFilter(myFilter);
    dataView.endUpdate();

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
    if (value === null || value === undefined || dataContext === undefined) { return ''; }
    return '<span class="severity ' + value + '">' + value + '</span>';
  }

  function myFilter(item, args) {
    // console.log(eval(args.filterScript));
    // return eval(args.filterScript);
    return true; // for development(no filter)
  }

  function updateFilter() {
    console.log('filter update');
    dataView.setFilterArgs({
      filterScript: filterScript,
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
  // add exclude filter

  //addボタン押下時の処理
  $('#add-exclude-button').on('click', function () {
    var filterText = '';
    var items;
    var radiobuttonStatus = $('input[name="exclude_input"]:checked').val();
    console.log(radiobuttonStatus);
    var filterStatus = $('#exclude-select').val();
    console.log(filterStatus);
    var conditions;
    if (radiobuttonStatus === 'AND') {
      //andを表すものを追加
      // console.log($('#exclude li').length);
      if ($('#exclude li').length === 0) {
        // return alert('NewGroup is not defined');
        return;
      }
      // conditions = '<button>' + radiobuttonStatus + '</button>';
      conditions = '<span class="label and">AND</span>';
    } else {
      //orを表すものを追加?
      conditions = '';
    }


    if (filterStatus === 'Message') {
      $('#exclude').append(function () {
        return '<li class="message" name="filter">'
          + '<div class="column">'
          + conditions
          + '<div class="name">'
          // + conditions
          + '<div class="parts-set checkbox"><input type="checkbox" name="isEffective" class="isEffective"><label for="">Message</label></div>'
          + '</div>'
          + '<div class="data">'
          + '<div class="parts-set textbox"><input type="text" class="message-value"placeholder="Message Text"></div>'
          + '<div class="parts-set checkbox"><input type="checkbox"><label for="">Regax</label></div>'
          + '</div>'
          + '</div>'
          + '<div class="delete">'
          + '<a href="" class="icon"><i class="material-icons">remove_circle</i></a>'
          + '</div>'
          + '</li>';
      });
    }


    if (filterStatus === 'Severity') {
      $('#exclude').append(function () {
        return '<li class="severities" name="filter">'
          + '<div class="column">'
          + conditions
          + '<div class="name">'
          // + conditions
          + '<div class="parts-set checkbox"><input type="checkbox" name="isEffective" class="isEffective"><label for="">Severities</label></div>'
          + '</div>'
          + '<div class="data">'
          + '<ul class="chips">'
          + '<li><a href="#" class="severities-select-button" data-value="Debug">Debug</a></li>'
          + '<li><a href="#" class="severities-select-button" data-value="Info">Info</a></li>'
          + '<li><a href="#" class="severities-select-button" data-value="Warn">Warn</a></li>'
          + '<li><a href="#" class="severities-select-button" data-value="Error">Error</a></li>'
          + '<li><a href="#" class="severities-select-button" data-value="Fatal">Fatal</a></li>'
          + '</ul>'
          + '</div>'
          + '</div>'
          + '<div class="delete">'
          + '<a href="" class="icon"><i class="material-icons">remove_circle</i></a>'
          + '</div>'
          + '</li>';
      });
    }

    //TODO: 中身を更新する処理
    if (filterStatus === 'Node') {
      items = dataView.getItems();
      console.log(items);
      var nodeList = '';
      filterText = '';
      _.each(items, function (item, index) {
        nodeList = nodeList.concat(item.Node);
        nodeList = nodeList + ',';
      });
      nodeList = nodeList.substr(0, nodeList.length - 1);
      nodeList = nodeList.split(',');
      nodeList = nodeList.filter(function (node, idx, self) {
        return self.indexOf(node) === idx;
      });
      console.log(nodeList);
      filterText = '<li class="node" name="filter">'
        + '<div class="column">'
        + conditions
        + '<div class="name">'
        + '<div class="parts-set checkbox"><input type="checkbox" name="isEffective" class="isEffective"><label for="">Node</label></div>'
        + '</div>'
        + '<div class="data">'
        + '<select class="node-value" type="select">';

      _.each(nodeList, function (node, index) {
        filterText = filterText + '<option value="' + node + '">' + node + '</option>';
      });

      filterText = filterText + '</select >'
        + '</div >'
        + '</div >'
        + '<div class="delete">'
        + '<a href="" class="icon"><i class="material-icons">remove_circle</i></a>'
        + '</div>'
        + '</li >';

      $('#exclude').append(function () {
        return filterText;
      });
    }


    if (filterStatus === 'Stamp') {
      $('#exclude').append(function () {
        return '<li class="time" name="filter">'
          + '<div class="column">'
          + conditions
          + '<div class="name">'
          + '<div class="parts-set checkbox"><input type="checkbox" name="isEffective" class="isEffective"><label for="">from time range</label>'
          + '</div>'
          + '</div>'
          + '<div class="data">'
          + '<select type="select">'
          + '<option value="0">Time Selected</option>'
          + '<option value="1">1</option>'
          + '<option value="2">2</option>'
          + '<option value="3">3</option>'
          + '</select>'
          // + '<input type="date" name="stamp">'
          // + '<input type="time" name="stamp" step="0.01">'
          + '<input type="checkbox">'
          + '<select type="select" disabled>'
          + '<option value="0">Time Selected</option>'
          + '<option value="1">1</option>'
          + '<option value="2">2</option>'
          + '<option value="3">3</option>'
          + '</select>'
          + '</div>'
          + '</div>'
          + '<div class="delete">'
          + '<a href="" class="icon"><i class="material-icons">remove_circle</i></a>'
          + '</div>'
          + '</li>';
      });
    }


    //TODO: 中身を更新する処理
    if (filterStatus === 'Topics') {
      items = dataView.getItems();
      var topicList = '';
      filterText = '';
      _.each(items, function (item, index) {
        topicList = topicList.concat(item.Topics);
        topicList = topicList + ',';
      });
      topicList = topicList.substr(0, topicList.length - 1);
      topicList = topicList.split(',');

      topicList = topicList.filter(function (topic, idx, self) {
        return self.indexOf(topic) === idx;
      });
      console.log(topicList);
      filterText = '<li class="topics" name="filter">'
        + '<div class="column">'
        + conditions
        + '<div class="name">'
        // + conditions
        + '<div class="parts-set checkbox"><input type="checkbox" name="isEffective" class="isEffective"><label for="">Topics</label></div>'
        + '</div >'
        + '<div class="data">'
        + '<select class="topic-value" type="select">';

      _.each(topicList, function (topic, index) {
        filterText = filterText + '<option value="' + topic + '">' + topic + '</option>';
      });

      filterText = filterText + '</select>'
        + '</div>'
        + '</div>'
        + '<div class="delete">'
        + '<a href="" class="icon"><i class="material-icons">remove_circle</i></a>'
        + '</div>'
        + '</li >';

      $('#exclude').append(function () {
        return filterText;
      });
    }


    if (filterStatus === 'Location') {
      $('#exclude').append(function () {
        return '<li class="location" name="filter">'
          + '<div class="column">'
          + conditions
          + '<div class="name">'
          + '<div class="parts-set checkbox"><input type="checkbox" name="isEffective" class="isEffective"><label for="">Location</label></div>'
          + '</div>'
          + '<div class="data">'
          + '<div class="parts-set textbox"><input type="text" class="location-value" placeholder="Location Text"></div>'
          + '<div class="parts-set checkbox"><input type="checkbox"><label for="">Regax</label></div>'
          + '</div>'
          + '</div>'
          + '<div class="delete">'
          + '<a href="" class="icon"><i class="material-icons">remove_circle</i></a>'
          + '</div>'
          + '</li>';
      });
    }

  });

  //除外条件のアクティブクラス書き換え
  $('#exclude').on('click', 'input[name = "filter"]', function (e) {
    // e.preventDefault();
    var className = this.getAttribute('class');
    this.setAttribute('class', className + ' active');

    // $('#exclude').trigger('change');
  });


  ////////////////////////////////////////
  // change exclude filter

  //Severityのクラス書き換え
  $('#exclude').on('click', '.severities-select-button', function (e) {
    e.preventDefault();
    var className = this.getAttribute('class');
    if (className === 'severities-select-button') {
      this.setAttribute('class', 'severities-select-button active');
    } else {
      this.setAttribute('class', 'severities-select-button');
    }
    $('#exclude').trigger('change');
  });

  function getElementName(element) {
    var elementName = element.attr('class');
    switch (elementName) {
      case 'message':
        elementName = 'Message';
        break;

      case 'severities':
        elementName = 'Severity';
        break;

      case 'node':
        elementName = 'Node';
        break;

      case 'time':
        elementName = 'Stamp';
        break;

      case 'topics':
        elementName = 'Topics';
        break;

      case 'location':
        elementName = 'Location';
        break;
    }
    return elementName;
  }

  function getFilterValue(elementName, $elm) {
    var filterValue;
    switch (elementName) {
      case 'Message':
        filterValue = $elm.find('.message-value').val();
        break;

      case 'Severity':
        filterValue = $elm.find('.severities-select-button.active');
        console.log(filterValue);
        break;

      case 'Node':
        filterValue = $elm.find('.node-value').val();
        break;

      case 'Stamp':
        filterValue = $elm.find('.topic-value').val();
        break;

      case 'Topics':
        filterValue = $elm.find('.topic-value').val();
        break;

      case 'Location':
        filterValue = $elm.find('.location-value').val();
        break;
    }
    return filterValue;
  }

  //excludeフィルタの中の値に変化があった時の処理
  $('#exclude').on('change', function (e) {
    e.preventDefault();

    var excludeScript = '';
    var excludeDetail = $('#exclude li');
    var isParentEffective = true;
    var isParent = false;
    var isTopic = false;
    var isSeverity = false;
    var headScript = 'function filter(item){';
    var $checkbox;
    var elementName;
    var filterValue;

    // var isSelectType = true;
    _.each(excludeDetail, function (nativeElm, index) {
      // console.log(index + ':' + nativeElm);
      var $elm = $(nativeElm);
      var isNewGroup = ($elm.find('span.label.and').length === 0);
      // console.log('isNewGroup:' + isNewGroup);


      if (isNewGroup) {
        // isParent = false;

        //OR時の処理
        $checkbox = $($elm.find('input.isEffective'));
        isParentEffective = $checkbox.prop('checked');

        if (isParentEffective) {
          if (index !== 0 && isParent) {
            excludeScript = excludeScript + '){ return false; }';
          }
          // console.log('parent in!!!!');

          isParent = true;
          //選択系
          //node
          elementName = getElementName($elm);
          filterValue = getFilterValue(elementName, $elm);
          // console.log(elementName);
          // console.log(filterValue);

          switch (elementName) {
            case 'Message':
              excludeScript = excludeScript + ' if(' + 'item["Message"]' + '.indexOf("' + filterValue + '") !== -1';
              break;

            case 'Severity':
              if (!isSeverity) {
                isSeverity = true;
                headScript = headScript + ' var severity=[];';

                // for (var key in filterValue) {
                _.each(filterValue, function (value, index) {
                  console.log($(value).data());
                  var severityValue = $(value).data();
                  console.log(severityValue['value']);
                  headScript = headScript + ' severity.push("' + severityValue['value'] + '"); ';
                });
              }
              excludeScript = excludeScript + ' if( severity.indexOf(' + 'item["Severity"]' + ') !== -1';
              break;

            case 'Node':
              excludeScript = excludeScript + ' if(' + 'item["Node"]' + '.indexOf("' + filterValue + '") !== -1';
              break;

            case 'Stamp':
              break;

            case 'Topic':
              if (!isTopic) {
                isTopic = true;
                headScript = headScript + ' var topicName = item["Topics"].split(",");';
              }
              excludeScript = excludeScript + ' if(' + 'topicName' + '.indexOf("' + filterValue + '") !== -1';
              break;

            case 'Location':
              excludeScript = excludeScript + ' if(' + 'item["Location"]' + '.indexOf("' + filterValue + '") !== -1';
              break;
          }
        }

      } else {
        if (isParentEffective) {
          $checkbox = $($elm.find('input.isEffective'));
          var isChildEffective = $checkbox.prop('checked');
          //AND時の処理
          elementName = getElementName($elm);
          filterValue = getFilterValue(elementName, $elm);

          if (isChildEffective) {

            switch (elementName) {
              case 'Message':
                excludeScript = excludeScript + ' &&' + 'item["' + elementName + '"]' + '.indexOf("' + filterValue + '") !== -1';
                break;

              case 'Severity':
                if (!isSeverity) {
                  isSeverity = true;
                  headScript = headScript + ' var severity=[];';

                  // for (var key in filterValue) {
                  _.each(filterValue, function (value, index) {
                    console.log(value);
                    var severityValue = $(value).data();
                    headScript = headScript + ' severity.push("' + severityValue['value'] + '"); ';
                  });
                }
                excludeScript = excludeScript + ' && severity.indexOf(' + 'item["' + elementName + '"]' + ') !== -1';
                break;

              case 'Node':
                excludeScript = excludeScript + ' &&' + 'item["' + elementName + '"]' + '.indexOf("' + filterValue + '") !== -1';
                break;

              case 'Stamp':

                break;

              case 'Topics':
                if (!isTopic) {
                  isTopic = true;
                  headScript = headScript + ' var topicName = item["Topics"].split(",");';
                }
                excludeScript = excludeScript + ' &&' + ' topicName' + '.indexOf("' + filterValue + '") !== -1';
                break;

              case 'Location':
                excludeScript = excludeScript + ' &&' + 'item["Location"]' + '.indexOf("' + filterValue + '") !== -1';
                break;
            }

          }

        }
      }

      console.log('isParentEffective:' + isParentEffective);
    });
    if (!isParent) {
      excludeScript = excludeScript + 'return true;} filter(item);';
    } else {
      excludeScript = excludeScript + '){ return false; } return true ;} filter(item);';
    }


    filterScript = headScript + excludeScript;
    console.log(filterScript);
    updateFilter();
  });


  ////////////////////////////////////////
  // start screen

  initScreen();
});