import { EventEmitter } from 'events';

const eventEmitter = new EventEmitter();

export function emitLogEvent(logData: any) {
    eventEmitter.emit('log', logData);
}

export function subscribeToLogEvents(callback: (logData: any) => void) {
    eventEmitter.on('log', callback);
}
