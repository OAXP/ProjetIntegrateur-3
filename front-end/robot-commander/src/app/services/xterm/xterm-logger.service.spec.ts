import { XtermLoggerService } from './xterm-logger.service';

describe('XtermLoggerService', () => {
  let service: XtermLoggerService;

  beforeEach(() => {
    service = new XtermLoggerService();
  });

  it('should emit formatted message when logToXterm is called', (done) => {
    const testMessage = 'Test message';
    const testType = 'info';
    service.messages$.subscribe((data) => {
      expect(data.message).toContain(testMessage);
      expect(data.message).toContain(testType.toUpperCase());
      expect(data.type).toEqual(testType);
      // Vérifier le format de temps pourrait être plus complexe, donc cela pourrait être omis ou simplifié
      done();
    });

    service.logToXterm(testMessage, testType);
  });

  it('should return correct ANSI color code for message types', () => {
    expect(service.getAnsiColorCode('success')).toEqual('\x1b[32m'); // Vert
    expect(service.getAnsiColorCode('error')).toEqual('\x1b[31m'); // Rouge
    expect(service.getAnsiColorCode('warn')).toEqual('\x1b[33m'); // Jaune
    expect(service.getAnsiColorCode('info')).toEqual('\x1b[0m'); // Réinitialiser
  });
});
