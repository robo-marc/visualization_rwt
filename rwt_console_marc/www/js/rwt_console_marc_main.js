function requiredFieldValidator(value) {
  if (value === null || value === undefined || !value.length) {
    return { valid: false, msg: 'This is a required field' };
  } else {
    return { valid: true, msg: null };
  }
}

$(function () {


  var dataView;
  var grid;
  var data = [];
  var columns = [
    { id: '#', name: '#', field: '#', sortable: true },
    { id: 'Message', name: 'Message', width: 240, field: 'Message', sortable: true },
    { id: 'Severity', name: 'Severity', field: 'Severity', sortable: true },
    { id: 'Node', name: 'Node', field: 'Node', sortable: true },
    { id: 'Stamp', name: 'Stamp', field: 'Stamp', width: 240, sortable: true },
    { id: 'Topics', name: 'Topics', field: 'Topics', width: 240, sortable: true },
    { id: 'Location', name: 'Location', field: 'Location', width: 240, sortable: true },
    { id: 'Number', name: 'Number', field: 'Number', width: -1, maxWidth: -1, minWidth: -1, resizable: false, headerCssClass: 'hidden', sortable: true }
  ];

  var options = {
    editable: true,
    enableAddRow: false,
    enableCellNavigation: true,
    asyncEditorLoading: false
  };

  function myFilter(item) {
    var percentCompleteThreshold = 0;
    var searchString = '';
    if (item['percentComplete'] < percentCompleteThreshold) {
      return false;
    }

    if (searchString !== '' && item['title'].indexOf(searchString) === -1) {
      return false;
    }

    if (item.parent !== null) {
      var parent = data[item.parent];

      while (parent) {
        if (parent._collapsed || (parent['percentComplete'] < percentCompleteThreshold) || (searchString !== '' && parent['title'].indexOf(searchString) === -1)) {
          return false;
        }
        parent = data[parent.parent];
      }
    }
    return true;
  }

  function percentCompleteSort(a, b) {
    return a['percentComplete'] - b['percentComplete'];
  }

  dataView = new Slick.Data.DataView({ inlineFilters: true });
  grid = new Slick.Grid('#myGrid', dataView, columns, options);
  var list = [];
  var isSubscribe = 1;
  var count = 1;
  var isAsc = false;

  $(function () {
    // subscribe topic
    var ros = new ROSLIB.Ros();
    ros.autoConnect();

    $('#pause-button').show();
    $('#resume-button').hide();

    // display start
    function startDrawing() {
      var sub = null;
      sub = new ROSLIB.Topic({
        ros: ros,
        name: '/rosout_agg',
        messageType: 'rosgraph_msgs/Log'
      });
      sub.subscribe(function (msg) {
        if (isSubscribe === 1) {
          var mes = msg.msg;
          var severityNumber = msg.level;
          var severity = getErrorLevel(msg.level);
          var node = msg.name;
          var time = getTimestamp(msg.header.stamp.secs, msg.header.stamp.nsecs);
          var regular = ('0000000000' + msg.header.stamp.nsecs).slice(-9);
          var stamp = time;
          var topics = msg.topics.join(',');
          var location = msg.file + ':' + msg.function + ':' + msg.line;

          var associationItem = {
            id: count,
            indent: 0,
            '#': '#' + count,
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

          if (count >= 20001) {
            dataView.deleteItem(count - 20000);
          }

          list.push(associationItem);

          if (isAsc === false) {
            dataView.insertItem(0, associationItem);
            grid.invalidate();
          }

          if (isAsc === true) {
            dataView.insertItem(count, associationItem);
            grid.invalidate();
          }

          count++;

          var txt = 'Displaying <h3>' + list.length + '</h3> messages';
          // document.getElementById('NumberOfMessage').innerHTML = txt;
        }
      });
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

    // get timestamp
    function getTimestamp(secs, nsecs) {
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

    // clear list
    function clearList() {
      list.length = 0;
      data = [];
      count = 1;
      dataView.setItems(data);
      // dataView.setFilter(myFilter);
      grid.invalidate();
      // document.getElementById('NumberOfMessage').innerHTML = 'Displaying <h3>0</h3> messages';
    }

    // clear list
    $('#clear-button').click(function (e) {
      e.preventDefault();
      clearList();
    });

    // CSV download
    $('#download-button').click(function (e) {
      // do not preventDefault.

      var arr = [];
      var itemList = _.cloneDeep(list);
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
      if (!(csvData === '')) {
        csvData = csvData.slice(0, -1); + '"' + '\n';
      }
      var csvText = 'message;severity;node;stamp;topics;location\n' + csvData;
      var blob = new Blob([csvText], { 'type': 'text/plain' });
      if (window.navigator.msSaveBlob) {
        window.navigator.msSaveBlob(blob, 'rwt_console_marc.csv');
        window.navigator.msSaveOrOpenBlob(blob, 'rwt_console_marc.csv');
        console.log('aaa');
      } else {
        console.log(blob);
        // document.getElementById('download').href = window.URL.createObjectURL(blob);
        $('#download-button').attr('href', window.URL.createObjectURL(blob));
      }
      delete csvData;
    });

    // CSV load
    // file change
    $('#load-button').on('click', function (e) {
      e.preventDefault();
      if (list.length === 0 && isSubscribe === 0) {
        $('#load').click();
      }
    })

    // $('#load').on('change', function (evt) {
    //   $load = $(this);
    //   var file = $load.val();
    // });

    var fileObj = document.getElementById('load');
    fileObj.addEventListener('change', function (evt) {
      var test = [];
      dataView.setItems(test);
      var file = evt.target.files[0];
      console.log(file);

      // TODO CSV file only
      // if (!file.name.match('.csv$')) {
      //   alert('.csv file only');
      //   return;
      // }

      // read text
      var reader = new FileReader();
      reader.readAsText(file);

      reader.onload = function () {
        // reader.result.replace('"', '');
        // reader.result = reader.result.replace(/"/g, '');
        console.log(reader.result);
        var cols = reader.result.split('\n');
        console.log(cols);
        var data = [];
        var keep;
        for (var i = 0; i < cols.length; i++) {
          // keep = cols[i].substring();
          keep = cols[i].substr(1, cols[i].length - 2);
          data[i] = keep.split('";"');
        }
        console.log('---- data ----');
        console.log(data);
        gridList(data);
      };
    });

    // grid List
    function gridList(dataList) {
      // console.log(dataList);

      // var data = [];

      // for (var i = 1; i < dataList.length - 2; i++) {
      //   if (dataList[i]) {
      //     list[i] = {
      //       // #: dataList,
      //       // value: dataList[i].Message,
      //       // id: count,
      //       // indent: 0,

      //       '#': '#' + dataList[i][6],
      //       Message: dataList[i][0],
      //       // SeverityNumber: dataList[i][4],
      //       Severity: dataList[i][1],
      //       Node: dataList[i][2],
      //       Stamp: dataList[i][3],
      //       // RawTime: msg.header.stamp.secs + '.' + regular,
      //       Topics: dataList[i][4],
      //       Location: dataList[i][5],
      //       Number: dataList[i][6]
      //     };
      //   }
      // }
      // var keep = dataList.length;
      dataList = dataList.slice().reverse();
      var number = 1;
      for (var i = 1; i < dataList.length - 1; i++) {
        if (dataList[i]) {
          var time = dataList[i][3].split('.');
          var secs = time[0];
          var nsecs = time[1];
          // var mes = msg.msg;
          // var severityNumber = msg.level;
          // var severity = getErrorLevel(msg.level);
          // var node = msg.name;
          // var time = getTimestamp(msg.header.stamp.secs, msg.header.stamp.nsecs);
          // var regular = ('0000000000' + msg.header.stamp.nsecs).slice(-9);
          // var stamp = time;
          // var topics = msg.topics.join(',');
          // var location = msg.file + ':' + msg.function + ':' + msg.line;


          var associationItem = {
            id: i,
            indent: 0,
            '#': '#' + number,
            Message: dataList[i][0],
            SeverityNumber: dataList[i][1],
            Severity: getErrorLevel(parseInt(dataList[i][1], 10)),
            Node: dataList[i][2],
            Stamp: getTimestamp(secs, nsecs),
            // RawTime: msg.header.stamp.secs + '.' + regular,
            Topics: dataList[i][4],
            Location: dataList[i][5],
            // Number: dataList[i][6]
          };
          number++;
          // console.log(associationItem);

          if (count >= 20001) {
            dataView.deleteItem(count - 20000);
          }

          list.push(associationItem);

          if (isAsc === false) {
            dataView.insertItem(count, associationItem);
            grid.invalidate();
          }

          if (isAsc === true) {
            dataView.insertItem(0, associationItem);
            grid.invalidate();
          }
        }
      }
      // count++;

      var txt = 'Displaying <h3>' + list.length + '</h3> messages';
      // document.getElementById('NumberOfMessage').innerHTML = txt;

      // console.log(list);
      // var grid = new Slick.Grid('#myGrid', list, columns, options);
      // grid.onSort.subscribe(function (e, args) {
      //   grid.invalidateAllRows();
      //   grid.render();
      // });

    }

    // pause
    $('#pause-button').click(function (e) {
      e.preventDefault();
      isSubscribe = 0;
      $('#pause-button').hide();
      $('#resume-button').show();
    });

    // start
    $('#resume-button').click(function (e) {
      e.preventDefault();
      isSubscribe = 1;
      $('#pause-button').show();
      $('#resume-button').hide();
    });

    // sort
    var sortcol;
    grid.onSort.subscribe(function (e, args) {
      sortcol = args.sortCol.field;

      if (sortcol === '#') {
        sortcol = 'Stamp';
      }
      if (sortcol === 'Stamp') {
        isAsc = args.sortAsc;
        dataView.sort(comparer, isAsc);
        grid.invalidateAllRows();
        grid.render();
      }

      if (!(sortcol === 'Stamp')) {
        isAsc = args.sortAsc;
        dataView.sort(comparer, isAsc);
        dataView.sort(comparer2, isAsc);
        grid.invalidateAllRows();
        grid.render();
      }
    });

    function comparer(a, b) {
      var x = a[sortcol], y = b[sortcol];
      return (x === y ? 0 : (x > y ? 1 : -1));
    }

    function comparer2(a, b) {
      var x = a[sortcol], y = b[sortcol];
      var z = a['Stamp'], w = b['Stamp'];
      return (x === y ? (z > w ? 1 : -1) : 0);
    }



    startDrawing();

  });

  // var sample = function () {
  //   var indent = 0;
  //   var parents = [];
  //   var rowCount = 1;
  //   // prepare the data
  //   _.each(list, function (msg, index) {

  //     var d = (data[index] = {});
  //     var parent;

  //     if (parents.length > 0) {
  //       parent = parents[parents.length - 1];
  //     } else {
  //       parent = null;
  //     }
  //     d['id'] = 'id_' + index;
  //     d['indent'] = indent;
  //     d['parent'] = parent;
  //     d['#'] = '#' + rowCount;
  //     d['Message'] = msg.Message;
  //     d['Severity'] = severity;
  //     d['Node'] = msg.Node;
  //     d['Stamp'] = msg.Stamp;
  //     d['Topics'] = msg.Topics;
  //     d['Location'] = msg.Location;

  //     rowCount++;
  //   });


  //   // initialize the model
  //   dataView.beginUpdate();
  //   var sortData = data.slice().reverse();
  //   dataView.setItems(sortData);
  //   dataView.setFilter(myFilter);
  //   dataView.endUpdate();


  //   // initialize the grid
  //   grid = new Slick.Grid('#myGrid', dataView, columns, options);

  //   grid.onCellChange.subscribe(function (e, args) {
  //     dataView.updateItem(args.item.id, args.item);
  //   });

  //   // wire up the search textbox to apply the filter to the model
  //   $('#txtSearch').keyup(function (e) {
  //     Slick.GlobalEditorLock.cancelCurrentEdit();

  //     // clear on Esc
  //     if (e.which === 27) {
  //       this.value = '';
  //     }

  //     searchString = this.value;
  //     dataView.refresh();
  //   });
  // };

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


});