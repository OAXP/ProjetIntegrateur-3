import { ComponentFixture, TestBed } from '@angular/core/testing';
import { XtermComponent } from './xterm.component';
import { XtermLoggerService } from '../../services/xterm/xterm-logger.service';
import { SocketClientService } from 'src/app/services/socket-client/socket-client.service';
import { MatDialogRef, MAT_DIALOG_DATA } from '@angular/material/dialog';
import { CUSTOM_ELEMENTS_SCHEMA } from '@angular/core';
import { Observable, of } from 'rxjs';

class MockXtermLoggerService {
  messages$ = of({ message: "Test log" });
}

class MockSocketClientService {
  socket = {
    emit: jasmine.createSpy('emit'),
    on: jasmine.createSpy('on').and.callFake((eventName: string, callback: Function) => {
      if (eventName === 'mission_data') {
        callback({ log: "Mission log" });
      }
    })
  };
}

const mockDialogRef = {
  close: jasmine.createSpy('close')
};

const mockData = { missionId: 1 };

describe('XtermComponent', () => {
  let component: XtermComponent;
  let fixture: ComponentFixture<XtermComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ XtermComponent ],
      providers: [
        { provide: XtermLoggerService, useClass: MockXtermLoggerService },
        { provide: SocketClientService, useClass: MockSocketClientService },
        { provide: MatDialogRef, useValue: mockDialogRef },
        { provide: MAT_DIALOG_DATA, useValue: mockData }
      ],
      schemas: [CUSTOM_ELEMENTS_SCHEMA]
    }).compileComponents();

    fixture = TestBed.createComponent(XtermComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should close the dialog when closeDialog is called', () => {
    component.closeDialog();
    expect(mockDialogRef.close).toHaveBeenCalled();
  });
  
});

