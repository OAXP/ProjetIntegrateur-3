import { Injectable } from '@angular/core';
import { Subject } from 'rxjs';

@Injectable({
  providedIn: 'root',
})
export class XtermLoggerService {
  private messageStream = new Subject<{ message: string, type: string }>();

  public messages$ = this.messageStream.asObservable();

  public logToXterm(message: string, type: string = 'info') {
    const now = new Date();
    const time = [
      now.getHours().toString().padStart(2, '0'),
      now.getMinutes().toString().padStart(2, '0'),
      now.getSeconds().toString().padStart(2, '0'),
    ].join(':');
  
    const colorCode = this.getAnsiColorCode(type);
    const resetColor = '\x1b[0m';
    const formattedMessage = `${colorCode}${time} - [${type.toUpperCase()}] ${message}${resetColor}`;
    this.messageStream.next({ message: formattedMessage, type });
  }  
  
  getAnsiColorCode(type: string): string {
    switch (type) {
      case 'success':
        return '\x1b[32m'; // Vert
      case 'error':
        return '\x1b[31m'; // Rouge
      case 'warn':
        return '\x1b[33m'; // Jaune
      default:
        return '\x1b[0m'; // Réinitialiser
    }
  }
}