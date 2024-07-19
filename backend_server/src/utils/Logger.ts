import { emitLogEvent } from './EventEmitter';
import { FileService } from '../services/file.service';
export default class Logger {
    private readonly name: string;

    constructor(name: string, private readonly fileService: FileService) {
        this.name = name;
    }

    info(content: any) {
        let info = `${this.name} : ${content}`;
        emitLogEvent({ level: 'info', message: info });
        info = `${new Date().toJSON()} - ` + info;
        console.info(info);
        this.fileService.writeLog(info);
    }

    err(content: any, error?: any) {
        let errMsg = `${this.name} - ${new Date().toJSON()}: ${content}`;
        emitLogEvent({ level: 'error', message: errMsg, error });
        errMsg = `${new Date().toJSON()} - ` + errMsg;
        console.error(errMsg, error);
        this.fileService.writeLog(errMsg);
    }
}
