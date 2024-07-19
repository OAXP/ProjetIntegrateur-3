import {
  Component,
  ElementRef,
  ViewChild,
  Input,
  AfterViewInit,
  OnDestroy,
  Inject,
  Optional
} from '@angular/core';
import { Terminal } from 'xterm';
import { XtermLoggerService } from '../../services/xterm/xterm-logger.service';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';
import { SocketClientService } from 'src/app/services/socket-client/socket-client.service';

const INITIAL_ROWS = 19;
const INITIAL_COLS = 60;
const LINE_HEIGHT = 17;
const CHAR_WIDTH = 9;
const PADDING = 5;

@Component({
  selector: 'app-xterm',
  templateUrl: './xterm.component.html',
  styleUrls: ['./xterm.component.css'],
})
export class XtermComponent implements AfterViewInit, OnDestroy {
  @Input() isLocked: boolean = true;
  @ViewChild('terminal') terminalDiv!: ElementRef;
  @ViewChild('container') container!: ElementRef;
  private resizeObserver!: ResizeObserver;

  private term: Terminal;

  constructor(
    private xtermLoggerService: XtermLoggerService,
    private socketClient: SocketClientService,
    @Optional() public dialogRef: MatDialogRef<XtermComponent>,
    @Optional() @Inject(MAT_DIALOG_DATA) public data: any
  ) {
    this.term = new Terminal();
  }

  ngAfterViewInit() {
    this.term = new Terminal({
      cursorBlink: true,
      theme: {
        background: '#2b2b2b',
      },
    });
    this.term.resize(INITIAL_COLS, INITIAL_ROWS);
    this.term.open(this.terminalDiv.nativeElement);
    this.subscribeToTerminalEvents();

    this.resizeObserver = new ResizeObserver((entries) => {
      for (let entry of entries) {
        this.term.resize(
          Math.round(entry.contentRect.width / CHAR_WIDTH) - PADDING,
          Math.round(entry.contentRect.height / LINE_HEIGHT) - PADDING
        );
      }
    });
    this.resizeObserver.observe(this.container.nativeElement);

    if (this.data && this.data.missionId) {
      this.loadMissionData(this.data.missionId);
    }
  }

  ngOnDestroy() {
    if (this.term) {
      this.term.dispose();
    }
    if (this.resizeObserver) {
      this.resizeObserver.disconnect();
    }
  }

  subscribeToTerminalEvents() {
    this.xtermLoggerService.messages$.subscribe(({ message }) => {
      this.term.writeln(message);
    });
  }

  clearTerminal() {
    this.term.clear();
  }

  closeDialog(): void {
    if (this.dialogRef) {
      this.dialogRef.close();
    }
  }

  private loadMissionData(missionId: number) {
    this.socketClient.socket.on('mission_data', (data: any) => {
      const lines = data.log.split('\n');
      lines.forEach((line: string) => {
        this.term.writeln(line);
      });
    });
    this.socketClient.socket.emit('request_mission_data', missionId);
  }
}
