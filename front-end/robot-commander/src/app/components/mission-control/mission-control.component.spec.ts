import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MissionControlComponent } from './mission-control.component';
import { CommunicationService } from 'src/app/services/communication/communication.service';
import { of } from 'rxjs';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatCardModule } from '@angular/material/card';
import { NoopAnimationsModule } from '@angular/platform-browser/animations';

// Création d'un mock pour CommunicationService
class MockCommunicationService {
  startMission = jasmine.createSpy('startMission').and.returnValue(of({}));
  stopMission = jasmine.createSpy('stopMission').and.returnValue(of({}));
}

describe('MissionControlComponent', () => {
  let component: MissionControlComponent;
  let fixture: ComponentFixture<MissionControlComponent>;
  let mockCommunicationService: MockCommunicationService;

  beforeEach(async () => {
    mockCommunicationService = new MockCommunicationService();

    await TestBed.configureTestingModule({
      declarations: [MissionControlComponent],
      providers: [
        { provide: CommunicationService, useValue: mockCommunicationService }
      ],
      imports: [MatButtonModule, MatIconModule, MatCardModule, NoopAnimationsModule]
    }).compileComponents();

    fixture = TestBed.createComponent(MissionControlComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should call startMission on button click', () => {
    const startButton = fixture.nativeElement.querySelector('.start-button');
    startButton.click();
    fixture.detectChanges();

    expect(mockCommunicationService.startMission).toHaveBeenCalled();
  });

  it('should call stopMission on button click', () => {
    const returnButton = fixture.nativeElement.querySelector('.return-button');
    returnButton.click();
    fixture.detectChanges();

    expect(mockCommunicationService.stopMission).toHaveBeenCalled();
  });
});
