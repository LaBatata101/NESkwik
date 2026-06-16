package com.labatata.neskwik;

import android.content.ClipData;
import android.content.Intent;
import android.database.Cursor;
import android.net.Uri;
import android.provider.OpenableColumns;

import org.libsdl.app.SDLActivity;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;

public class NeskwikActivity extends SDLActivity {
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (data != null) {
            persistUriPermissions(data);
        }
        super.onActivityResult(requestCode, resultCode, data);
    }

    public String getDisplayNameForUri(String uriText) {
        if (uriText == null || !uriText.startsWith("content://")) {
            return null;
        }

        Uri uri = Uri.parse(uriText);
        Cursor cursor = null;
        try {
            cursor = getContentResolver().query(
                    uri,
                    new String[] { OpenableColumns.DISPLAY_NAME },
                    null,
                    null,
                    null
                );
            cursor.moveToFirst();
            return cursor.getString(cursor.getColumnIndex(OpenableColumns.DISPLAY_NAME));
        } finally {
            if (cursor != null) {
                cursor.close();
            }
        }
    }

    public int getCurrentScreenOrientation() {
        int rotation = getCurrentRotation();

        if (getNaturalOrientation() == SDL_ORIENTATION_LANDSCAPE) {
            rotation += 90;
        }

        switch (rotation % 360) {
            case 0:
                return SDL_ORIENTATION_PORTRAIT;
            case 90:
                return SDL_ORIENTATION_LANDSCAPE;
            case 180:
                return SDL_ORIENTATION_PORTRAIT_FLIPPED;
            case 270:
                return SDL_ORIENTATION_LANDSCAPE_FLIPPED;
            default:
                return SDL_ORIENTATION_UNKNOWN;
        }
    }

    private void persistUriPermissions(Intent data) {
        int permissionFlags = data.getFlags() &
            (Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION);

        getContentResolver().takePersistableUriPermission(data.getData(), permissionFlags);

        ClipData clipData = data.getClipData();
        if (clipData != null) {
            for (int i = 0; i < clipData.getItemCount(); i++) {
                getContentResolver().takePersistableUriPermission(clipData.getItemAt(i).getUri(), permissionFlags);
            }
        }
    }
}
