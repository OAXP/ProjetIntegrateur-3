import { TestBed } from '@angular/core/testing';
import { CommunicationService } from './communication.service';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';

describe('CommunicationService', () => {
  let service: CommunicationService;
  let httpTestingController: HttpTestingController;

  beforeEach(() => {
    TestBed.configureTestingModule({
      imports: [HttpClientTestingModule],
      providers: [CommunicationService]
    });

    service = TestBed.inject(CommunicationService);
    httpTestingController = TestBed.inject(HttpTestingController);
  });

  afterEach(() => {
    // Après chaque test, vérifiez qu'il n'y a pas de requêtes en suspens.
    httpTestingController.verify();
  });

  it('should call identify API and return its response', () => {
    const mockResponse = { success: true };
    service.identify('robot1').subscribe(response => {
      expect(response).toEqual(mockResponse);
    });
  
    const req = httpTestingController.expectOne('http://localhost:3000/api/command/identify');
    expect(req.request.method).toEqual('POST');
    expect(req.request.body).toEqual({ robot_id: 'robot1' });
    req.flush(mockResponse);
  });

  it('should handle errors on identify API call', () => {
    service.identify('robot1').subscribe({
      next: () => fail('should have failed with the 500 error'),
      error: (error) => {
        expect(error).toEqual('Something bad happened; please try again later.');
      }
    });
  
    const req = httpTestingController.expectOne('http://localhost:3000/api/command/identify');
    expect(req.request.method).toBe('POST');
  
    // Simuler une réponse d'erreur
    req.flush('Something bad happened', {
      status: 500,
      statusText: 'Internal Server Error'
    });
  });

  it('should call startMission API and return its response', () => {
    const mockResponse = { message: 'Mission started' };
    service.startMission().subscribe(response => {
      expect(response).toEqual(mockResponse);
    });
  
    const req = httpTestingController.expectOne('http://localhost:3000/api/command/start_mission');
    expect(req.request.method).toEqual('POST');
    req.flush(mockResponse);
  });
  
  it('should handle errors on startMission API call', () => {
    service.startMission().subscribe({
      next: () => fail('should have failed with the 500 error'),
      error: (error) => {
        expect(error).toEqual('Something bad happened; please try again later.');
      }
    });
  
    const req = httpTestingController.expectOne('http://localhost:3000/api/command/start_mission');
    req.flush('Error', { status: 500, statusText: 'Internal Server Error' });
  });
  
  it('should call stopMission API and return its response', () => {
    const mockResponse = { message: 'Mission stopped' };
    service.stopMission().subscribe(response => {
      expect(response).toEqual(mockResponse);
    });
  
    const req = httpTestingController.expectOne('http://localhost:3000/api/command/stop_mission');
    expect(req.request.method).toEqual('POST');
    req.flush(mockResponse);
  });
  
  it('should handle errors on stopMission API call', () => {
    service.stopMission().subscribe({
      next: () => fail('should have failed with the 500 error'),
      error: (error) => {
        expect(error).toEqual('Something bad happened; please try again later.');
      }
    });
  
    const req = httpTestingController.expectOne('http://localhost:3000/api/command/stop_mission');
    req.flush('Error', { status: 500, statusText: 'Internal Server Error' });
  });
  
  it('should call setInitPositions API with correct data and return its response', () => {
    const mockInitPositions = new Map([
      ['robot1', { init_position: { x: 0, y: 0 }, init_orientation: { x: 0, y: 0, z: 0, w: 1 } }]
    ]);
    const mockResponse = { success: true };
    service.setInitPositions(mockInitPositions).subscribe(response => {
      expect(response).toEqual(mockResponse);
    });
  
    const req = httpTestingController.expectOne('http://localhost:3000/api/info/init_positions');
    expect(req.request.method).toEqual('POST');
    expect(req.request.body).toEqual({
      'robot1': { init_position: { x: 0, y: 0 }, init_orientation: { x: 0, y: 0, z: 0, w: 1 } }
    });
    req.flush(mockResponse);
  });
  
  it('should handle errors on setInitPositions API call', () => {
    const mockInitPositions = new Map();
    service.setInitPositions(mockInitPositions).subscribe({
      next: () => fail('should have failed with the 500 error'),
      error: (error) => {
        expect(error).toEqual('Something bad happened; please try again later.');
      }
    });
  
    const req = httpTestingController.expectOne('http://localhost:3000/api/info/init_positions');
    req.flush('Error', { status: 500, statusText: 'Internal Server Error' });
  });  
  
});
